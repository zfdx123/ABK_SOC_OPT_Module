/*
 * abk_soc_opt.c — ABK SoC 功耗优化内核模块
 *
 * 内核态 cpufreq policy notifier，拦截用户空间 (perfd/thermal-engine)
 * 的频率修改并强制锁定上限。
 *
 * 温控由用户空间 AppOpt 通过 /sys/kernel/abk_soc_opt/freq_limits 控制，
 * 内核模块专注 cpufreq capping。
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

#define DRV_NAME          "abk_soc_opt"
#define MAX_CLUSTERS      4

/* ========================================================================
 * 模块参数
 * ===================================================================== */

static bool enabled = true;
module_param(enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable (1=on, 0=off)");

/* Per-cluster caps (kHz), comma-separated: little,mid,big,prime */
static int freq_limits[MAX_CLUSTERS] = { 0, 0, 0, 0 };
static int num_freq_limits;
module_param_array(freq_limits, int, &num_freq_limits, 0644);
MODULE_PARM_DESC(freq_limits,
    "Per-cluster max freq (kHz): little,mid,big,prime");

/* ========================================================================
 * 内部状态
 * ===================================================================== */

struct soc_cluster {
    unsigned int cpu;
    unsigned int hw_max;
    unsigned int cap;          /* 0 = no cap */
};

static struct soc_cluster clusters[MAX_CLUSTERS];
static int                num_clusters;
static struct mutex       lock;

/* ========================================================================
 * cpufreq policy notifier
 * ===================================================================== */

static int soc_cpufreq_notify(struct notifier_block *nb,
                               unsigned long action, void *data)
{
    struct cpufreq_policy *policy = data;
    int i;

    if (!enabled)
        return NOTIFY_DONE;

    /* 只处理 policy 更新，跳过创建/销毁 */
    if (action == CPUFREQ_CREATE_POLICY ||
        action == CPUFREQ_REMOVE_POLICY)
        return NOTIFY_DONE;

    for (i = 0; i < num_clusters; i++) {
        if (clusters[i].cpu != policy->cpu)
            continue;

        if (clusters[i].cap == 0 || clusters[i].cap >= clusters[i].hw_max)
            return NOTIFY_DONE;

        if (policy->max > clusters[i].cap) {
            pr_debug(DRV_NAME ": cpu%u max %u→%u kHz\n",
                     policy->cpu, policy->max, clusters[i].cap);
            policy->max = clusters[i].cap;

            if (policy->cur > clusters[i].cap)
                __cpufreq_driver_target(policy, clusters[i].cap,
                                        CPUFREQ_RELATION_H);
        }
        return NOTIFY_DONE;
    }

    return NOTIFY_DONE;
}

static struct notifier_block soc_nb = {
    .notifier_call = soc_cpufreq_notify,
};

/* ========================================================================
 * 初始化扫描
 * ===================================================================== */

static void soc_scan_and_apply(void)
{
    struct cpufreq_policy *policy;
    int idx = 0;
    int cpu;

    memset(clusters, 0, sizeof(clusters));

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

        clusters[idx].cpu    = cpu;
        clusters[idx].hw_max = policy->cpuinfo.max_freq;
        clusters[idx].cap    = 0;

        if (idx < num_freq_limits && freq_limits[idx] > 0)
            clusters[idx].cap = (unsigned int)freq_limits[idx];

        if (clusters[idx].cap > 0 && policy->max > clusters[idx].cap) {
            pr_info(DRV_NAME ": cluster%d cpu%u hw=%u cap=%u kHz\n",
                    idx, cpu, clusters[idx].hw_max, clusters[idx].cap);
            policy->max = clusters[idx].cap;
        }

        idx++;
        cpufreq_cpu_put(policy);
    }

    num_clusters = idx;
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
    if (kstrtoint(buf, 0, &val))
        return -EINVAL;
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
        if (sscanf(p, "%d%n", &val, &consumed) != 1)
            break;
        vals[n++] = val;
        p += consumed;
        while (*p == ',' || *p == ' ')
            p++;
    }
    if (n == 0)
        return -EINVAL;

    mutex_lock(&lock);
    for (int i = 0; i < n && i < num_clusters; i++) {
        clusters[i].cap = (unsigned int)vals[i];
        struct cpufreq_policy *policy = cpufreq_cpu_get(clusters[i].cpu);
        if (policy) {
            if (clusters[i].cap > 0 && policy->max > clusters[i].cap)
                policy->max = clusters[i].cap;
            cpufreq_cpu_put(policy);
        }
    }
    mutex_unlock(&lock);
    return count;
}
static struct kobj_attribute attr_freq_limits = __ATTR_RW(freq_limits);

/* cluster_info (ro) */
static ssize_t cluster_info_show(struct kobject *k, struct kobj_attribute *a,
                                  char *buf)
{
    int pos = 0;
    mutex_lock(&lock);
    for (int i = 0; i < num_clusters; i++)
        pos += scnprintf(buf + pos, PAGE_SIZE - pos,
                         "cluster%d: cpu%u hw_max=%ukHz cap=%ukHz\n",
                         i, clusters[i].cpu, clusters[i].hw_max,
                         clusters[i].cap);
    mutex_unlock(&lock);
    return pos;
}
static struct kobj_attribute attr_cluster_info = __ATTR_RO(cluster_info);

static struct attribute *soc_attrs[] = {
    &attr_enabled.attr,
    &attr_freq_limits.attr,
    &attr_cluster_info.attr,
    NULL,
};
static const struct attribute_group soc_attr_group = { .attrs = soc_attrs };

/* ========================================================================
 * 模块生命周期
 * ===================================================================== */

static int __init abk_soc_opt_init(void)
{
    int ret;

    pr_info(DRV_NAME ": loading\n");

    mutex_init(&lock);

    soc_kobj = kobject_create_and_add(DRV_NAME, kernel_kobj);
    if (!soc_kobj)
        return -ENOMEM;

    ret = sysfs_create_group(soc_kobj, &soc_attr_group);
    if (ret) {
        kobject_put(soc_kobj);
        return ret;
    }

    soc_scan_and_apply();

    ret = cpufreq_register_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    if (ret) {
        pr_err(DRV_NAME ": cpufreq notifier failed (%d)\n", ret);
        sysfs_remove_group(soc_kobj, &soc_attr_group);
        kobject_put(soc_kobj);
        return ret;
    }

    pr_info(DRV_NAME ": ready — %d clusters, sysfs at /sys/kernel/%s/\n",
            num_clusters, DRV_NAME);
    return 0;
}

static void __exit abk_soc_opt_exit(void)
{
    cpufreq_unregister_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);

    for (int i = 0; i < num_clusters; i++) {
        struct cpufreq_policy *policy = cpufreq_cpu_get(clusters[i].cpu);
        if (policy) {
            if (policy->max < clusters[i].hw_max)
                policy->max = clusters[i].hw_max;
            cpufreq_cpu_put(policy);
        }
    }

    sysfs_remove_group(soc_kobj, &soc_attr_group);
    kobject_put(soc_kobj);

    pr_info(DRV_NAME ": unloaded — all clusters restored\n");
}

module_init(abk_soc_opt_init);
module_exit(abk_soc_opt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AppOpt");
MODULE_DESCRIPTION("ABK SoC power optimization — cpufreq cap enforcement");
MODULE_VERSION("2.1");
