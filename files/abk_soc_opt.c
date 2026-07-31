/*
 * abk_soc_opt.c — ABK SoC 功耗优化内核模块 v2.2
 *
 * 内核态 cpufreq policy notifier，拦截用户空间频率修改并强制锁定上限。
 *
 * 新增 v2.2:
 *   - cap=0 自动离线该 cluster 全部核心（至少保留一个 cluster）
 *   - poll_ms 可配轮询周期，0=纯 notifier (默认)
 *   - 模块卸载时恢复所有离线核心
 *
 * Copyright (C) 2025 AppOpt
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/cpumask.h>
#include <linux/device.h>

#define DRV_NAME          "abk_soc_opt"
#define MAX_CLUSTERS      4

/* ========================================================================
 * 模块参数
 * ===================================================================== */

static bool enabled = true;
module_param(enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable (1=on, 0=off)");

/* Per-cluster caps (kHz), comma-separated: little,mid,big,prime
 * 0 = offline entire cluster (at least one cluster stays online) */
static int freq_limits[MAX_CLUSTERS] = { 0, 0, 0, 0 };
static int num_freq_limits;
module_param_array(freq_limits, int, &num_freq_limits, 0644);
MODULE_PARM_DESC(freq_limits,
    "Per-cluster max freq (kHz): 0=offline, >0=cap");

/* Polling period in ms. 0 = pure notifier mode (default). */
static unsigned int poll_ms;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms,
    "Polling interval in ms (0=notifier-only, default 0)");

/* ========================================================================
 * 内部状态
 * ===================================================================== */

struct soc_cluster {
    unsigned int first_cpu;
    unsigned int hw_max;
    unsigned int cap;          /* 0 = offlined */
    cpumask_var_t cpus;        /* all CPUs in this cluster */
    bool          mask_allocated;
    bool          offlined;
};

static struct soc_cluster clusters[MAX_CLUSTERS];
static int                num_clusters;
static int                clusters_online;  /* count of non-offlined clusters */
static struct mutex       lock;

/* polling */
static struct delayed_work poll_work;

/* ========================================================================
 * 核心下线/上线
 * ===================================================================== */

/* 下线一个 cluster 的全部核心。首次调用时若 CPU 不存在则静默跳过。 */
static void cluster_offline(struct soc_cluster *c)
{
    int cpu;

    if (c->offlined || c->cap > 0)
        return;

    /* 至少保留一个 cluster 在线 */
    if (clusters_online <= 1) {
        pr_warn(DRV_NAME ": refusing to offline last cluster (cpu%u)\n",
                c->first_cpu);
        return;
    }

    for_each_cpu(cpu, c->cpus) {
        if (cpu_online(cpu)) {
            struct device *dev = get_cpu_device(cpu);
            if (dev) { device_offline(dev); }
        }
    }
    c->offlined = true;
    clusters_online--;
    pr_info(DRV_NAME ": cluster cpu%u offlined\n", c->first_cpu);
}

/* 恢复一个 cluster */
static void cluster_online(struct soc_cluster *c)
{
    int cpu;

    if (!c->offlined)
        return;

    for_each_cpu(cpu, c->cpus) {
        if (!cpu_online(cpu)) {
            struct device *dev = get_cpu_device(cpu);
            if (dev) { device_online(dev); }
        }
    }
    c->offlined = false;
    clusters_online++;
    pr_info(DRV_NAME ": cluster cpu%u onlined\n", c->first_cpu);
}

/* ========================================================================
 * 频率强制
 * ===================================================================== */

/* 对一个 cluster 的 policy 应用 cap（不发 offline/online） */
static void cluster_apply_cap(struct soc_cluster *c)
{
    struct cpufreq_policy *policy;

    if (c->offlined)
        return;

    policy = cpufreq_cpu_get(c->first_cpu);
    if (!policy)
        return;

    if (c->cap > 0 && policy->max > c->cap) {
        pr_debug(DRV_NAME ": cpu%u max %u→%u kHz\n",
                 c->first_cpu, policy->max, c->cap);
        policy->max = c->cap;
        if (policy->cur > c->cap)
            __cpufreq_driver_target(policy, c->cap, CPUFREQ_RELATION_H);
    }

    cpufreq_cpu_put(policy);
}

/* 对所有 cluster 应用 cap */
static void enforce_all(void)
{
    int i;

    mutex_lock(&lock);
    for (i = 0; i < num_clusters; i++) {
        if (!clusters[i].offlined)
            cluster_apply_cap(&clusters[i]);
    }
    mutex_unlock(&lock);
}

/* ========================================================================
 * cpufreq policy notifier (主拦截路径)
 * ===================================================================== */

static int soc_cpufreq_notify(struct notifier_block *nb,
                               unsigned long action, void *data)
{
    struct cpufreq_policy *policy = data;
    int i;

    if (!enabled)
        return NOTIFY_DONE;

    if (action == CPUFREQ_CREATE_POLICY ||
        action == CPUFREQ_REMOVE_POLICY)
        return NOTIFY_DONE;

    mutex_lock(&lock);
    for (i = 0; i < num_clusters; i++) {
        if (clusters[i].first_cpu != policy->cpu)
            continue;
        if (!clusters[i].offlined)
            cluster_apply_cap(&clusters[i]);
        break;
    }
    mutex_unlock(&lock);

    return NOTIFY_DONE;
}

static struct notifier_block soc_nb = {
    .notifier_call = soc_cpufreq_notify,
};

/* ========================================================================
 * 轮询（兜底）
 * ===================================================================== */

static void poll_timer_cb(struct work_struct *work)
{
    enforce_all();
    if (poll_ms > 0)
        schedule_delayed_work(&poll_work, msecs_to_jiffies(poll_ms));
}

static void poll_start(void)
{
    if (poll_ms > 0)
        schedule_delayed_work(&poll_work, msecs_to_jiffies(poll_ms));
}

static void poll_stop(void)
{
    cancel_delayed_work_sync(&poll_work);
}

/* ========================================================================
 * 初始化扫描
 * ===================================================================== */

static void soc_scan_and_apply(void)
{
    struct cpufreq_policy *policy;
    int idx = 0;
    int cpu;

    memset(clusters, 0, sizeof(clusters));
    clusters_online = 0;

    for_each_possible_cpu(cpu) {
        if (idx >= MAX_CLUSTERS)
            break;

        policy = cpufreq_cpu_get(cpu);
        if (!policy)
            continue;

        if (policy->cpu != (unsigned int)cpu) {
            cpufreq_cpu_put(policy);
            continue;
        }

        /* 分配并拷贝 CPU 掩码 */
        if (!clusters[idx].mask_allocated) {
            if (!zalloc_cpumask_var(&clusters[idx].cpus, GFP_KERNEL)) {
                cpufreq_cpu_put(policy);
                break;
            }
            clusters[idx].mask_allocated = true;
        }
        cpumask_copy(clusters[idx].cpus, policy->related_cpus);

        clusters[idx].first_cpu = cpu;
        clusters[idx].hw_max    = policy->cpuinfo.max_freq;
        clusters[idx].cap       = 0;
        clusters[idx].offlined  = false;

        if (idx < num_freq_limits)
            clusters[idx].cap = (unsigned int)freq_limits[idx];
        /* cap < 0 → treat as 0 (offline) */
        if ((int)clusters[idx].cap < 0)
            clusters[idx].cap = 0;

        if (clusters[idx].cap > 0) {
            pr_info(DRV_NAME ": cluster%d cpu%u hw=%u cap=%u kHz\n",
                    idx, cpu, clusters[idx].hw_max, clusters[idx].cap);
            if (policy->max > clusters[idx].cap)
                policy->max = clusters[idx].cap;
            clusters_online++;
        }

        idx++;
        cpufreq_cpu_put(policy);
    }

    num_clusters = idx;

    /* 第二轮：offline cap==0 的 cluster */
    for (idx = 0; idx < num_clusters; idx++) {
        if (clusters[idx].cap == 0)
            cluster_offline(&clusters[idx]);
    }

    pr_info(DRV_NAME ": %d clusters, %d online\n",
            num_clusters, clusters_online);
}

/* ========================================================================
 * sysfs — /sys/kernel/abk_soc_opt/
 * ===================================================================== */

static struct kobject *soc_kobj;

/* enabled (rw) */
static ssize_t enabled_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", enabled);
}
static ssize_t enabled_store(struct kobject *k, struct kobj_attribute *a,
                              const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 0, &val)) return -EINVAL;
    enabled = !!val;
    return count;
}
static struct kobj_attribute attr_enabled = __ATTR_RW(enabled);

/* freq_limits (rw) */
static ssize_t freq_limits_show(struct kobject *k, struct kobj_attribute *a,
                                 char *buf)
{
    int pos = 0;
    mutex_lock(&lock);
    for (int i = 0; i < num_clusters; i++)
        pos += scnprintf(buf + pos, PAGE_SIZE - pos,
                         "%s%u", i ? "," : "", clusters[i].cap);
    mutex_unlock(&lock);
    pos += scnprintf(buf + pos, PAGE_SIZE - pos, "\n");
    return pos;
}

static ssize_t freq_limits_store(struct kobject *k, struct kobj_attribute *a,
                                  const char *buf, size_t count)
{
    int vals[MAX_CLUSTERS], n = 0;
    const char *p = buf;

    while (*p && n < MAX_CLUSTERS) {
        int val, consumed;
        if (sscanf(p, "%d%n", &val, &consumed) != 1) break;
        vals[n++] = val;
        p += consumed;
        while (*p == ',' || *p == ' ') p++;
    }
    if (n == 0) return -EINVAL;

    mutex_lock(&lock);
    for (int i = 0; i < n && i < num_clusters; i++) {
        unsigned int new_cap = (vals[i] < 0) ? 0 : (unsigned int)vals[i];
        bool was_online = !clusters[i].offlined;

        clusters[i].cap = new_cap;

        if (new_cap == 0 && was_online) {
            cluster_offline(&clusters[i]);
        } else if (new_cap > 0 && clusters[i].offlined) {
            cluster_online(&clusters[i]);
            cluster_apply_cap(&clusters[i]);
        } else if (new_cap > 0 && was_online) {
            cluster_apply_cap(&clusters[i]);
        }
    }
    mutex_unlock(&lock);
    return count;
}
static struct kobj_attribute attr_freq_limits = __ATTR_RW(freq_limits);

/* poll_ms (rw) */
static ssize_t poll_ms_show(struct kobject *k, struct kobj_attribute *a,
                             char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%u\n", poll_ms);
}
static ssize_t poll_ms_store(struct kobject *k, struct kobj_attribute *a,
                              const char *buf, size_t count)
{
    unsigned int val;
    if (kstrtouint(buf, 0, &val)) return -EINVAL;

    mutex_lock(&lock);
    poll_stop();
    poll_ms = val;
    poll_start();
    mutex_unlock(&lock);

    pr_info(DRV_NAME ": poll_ms = %u\n", poll_ms);
    return count;
}
static struct kobj_attribute attr_poll_ms = __ATTR_RW(poll_ms);

/* cluster_info (ro) */
static ssize_t cluster_info_show(struct kobject *k, struct kobj_attribute *a,
                                  char *buf)
{
    int pos = 0;
    mutex_lock(&lock);
    for (int i = 0; i < num_clusters; i++)
        pos += scnprintf(buf + pos, PAGE_SIZE - pos,
                         "cluster%d: cpu%u hw=%ukHz cap=%ukHz %s\n",
                         i, clusters[i].first_cpu, clusters[i].hw_max,
                         clusters[i].cap,
                         clusters[i].offlined ? "offline" : "online");
    mutex_unlock(&lock);
    return pos;
}
static struct kobj_attribute attr_cluster_info = __ATTR_RO(cluster_info);

static struct attribute *soc_attrs[] = {
    &attr_enabled.attr,
    &attr_freq_limits.attr,
    &attr_poll_ms.attr,
    &attr_cluster_info.attr,
    NULL,
};
static const struct attribute_group soc_attr_group = { .attrs = soc_attrs };

/* ========================================================================
 * 模块生命周期
 * ===================================================================== */

static void restore_all_clusters(void)
{
    int i;

    for (i = 0; i < num_clusters; i++) {
        if (clusters[i].offlined)
            cluster_online(&clusters[i]);

        struct cpufreq_policy *policy = cpufreq_cpu_get(clusters[i].first_cpu);
        if (policy) {
            if (policy->max < clusters[i].hw_max)
                policy->max = clusters[i].hw_max;
            cpufreq_cpu_put(policy);
        }

        if (clusters[i].mask_allocated)
            free_cpumask_var(clusters[i].cpus);
    }
}

static int __init abk_soc_opt_init(void)
{
    int ret;

    pr_info(DRV_NAME ": loading v2.2\n");

    mutex_init(&lock);

    soc_kobj = kobject_create_and_add(DRV_NAME, kernel_kobj);
    if (!soc_kobj) return -ENOMEM;

    ret = sysfs_create_group(soc_kobj, &soc_attr_group);
    if (ret) { kobject_put(soc_kobj); return ret; }

    INIT_DELAYED_WORK(&poll_work, poll_timer_cb);

    soc_scan_and_apply();

    ret = cpufreq_register_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    if (ret) {
        pr_err(DRV_NAME ": cpufreq notifier failed (%d)\n", ret);
        restore_all_clusters();
        sysfs_remove_group(soc_kobj, &soc_attr_group);
        kobject_put(soc_kobj);
        return ret;
    }

    poll_start();

    pr_info(DRV_NAME ": ready — %d clusters (%d online), poll=%ums, sysfs=/sys/kernel/%s/\n",
            num_clusters, clusters_online, poll_ms, DRV_NAME);
    return 0;
}

static void __exit abk_soc_opt_exit(void)
{
    poll_stop();
    cpufreq_unregister_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    restore_all_clusters();
    sysfs_remove_group(soc_kobj, &soc_attr_group);
    kobject_put(soc_kobj);

    pr_info(DRV_NAME ": unloaded — all cores restored\n");
}

module_init(abk_soc_opt_init);
module_exit(abk_soc_opt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AppOpt");
MODULE_DESCRIPTION("ABK SoC power optimization — cpufreq cap + core offlining + polling");
MODULE_VERSION("2.2");
