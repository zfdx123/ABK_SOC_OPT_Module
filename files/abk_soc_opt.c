/*
 * abk_soc_opt.c — ABK SoC 功耗优化内核模块
 *
 * abk_soc_opt ：
 *   - cpufreq policy notifier 内核态拦截频率修改
 *   - /sys/kernel/abk_soc_opt/ 动态控制接口
 *   - Per-cluster 频率上限 + 温控自动降频
 *   - 模块参数 insmod 即可用，也支持运行时 sysfs 修改
 *
 * Copyright (C) 2026 AppOpt
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mutex.h>
#include <linux/thermal.h>

#define DRV_NAME          "abk_soc_opt"
#define MAX_CLUSTERS      4
#define THERMAL_ZONE_MAX  24
#define DEFAULT_THERMAL_TEMP 75000   /* 75 C in milli-Celsius */

/* ========================================================================
 * 模块参数 — insmod / modprobe 时设置
 * ===================================================================== */

static bool enabled = true;
module_param(enabled, bool, 0644);
MODULE_PARM_DESC(enabled, "Enable SoC optimization (1=on)");

/* Per-cluster frequency caps (kHz), comma-separated:
 *   freq_limits=2000000,2200000,2600000,2800000
 * 顺序: little → mid → big → prime
 */
static int freq_limits[MAX_CLUSTERS] = { 0, 0, 0, 0 };
static int num_freq_limits;
module_param_array(freq_limits, int, &num_freq_limits, 0644);
MODULE_PARM_DESC(freq_limits,
    "Per-cluster max freq (kHz): little,mid,big,prime");

/* Thermal throttle frequency (kHz), applied to all clusters when overheating.
 * 0 = disable thermal throttle.
 */
static int thermal_limit;
module_param(thermal_limit, int, 0644);
MODULE_PARM_DESC(thermal_limit,
    "Thermal throttle freq (kHz) for all clusters");

/* Thermal threshold in milli-Celsius (default 75000 = 75°C).
 * Temperature above this triggers thermal_limit.
 */
static int thermal_temp = DEFAULT_THERMAL_TEMP;
module_param(thermal_temp, int, 0644);
MODULE_PARM_DESC(thermal_temp,
    "Thermal threshold in milli-Celsius (default 75000 = 75C)");

/* ========================================================================
 * 内部状态
 * ===================================================================== */

struct soc_cluster {
    unsigned int cpu;          /* policy CPU id */
    unsigned int hw_max;       /* hardware max (cpuinfo.max_freq) */
    unsigned int cap;          /* normal cap, 0 = no cap */
    unsigned int saved_cap;    /* saved before thermal override */
    bool         thermal;      /* currently thermally throttled */
};

static struct soc_cluster clusters[MAX_CLUSTERS];
static int                num_clusters;
static DEFINE_MUTEX       lock;

/* ========================================================================
 * 温控
 * ===================================================================== */

static int read_soc_temp(void)
{
    int max_temp = 0;

    for (int i = 0; i < THERMAL_ZONE_MAX; i++) {
        struct thermal_zone_device *tz;
        int temp;

        tz = thermal_zone_get_zone_by_name_id("", i);
        if (!tz || IS_ERR(tz))
            break;

        if (thermal_zone_get_temp(tz, &temp))
            continue;

        /* only CPU / SoC / GPU / skin / PA / XO zones */
        if (strstr(tz->type, "cpu")  || strstr(tz->type, "soc")  ||
            strstr(tz->type, "gpu")  || strstr(tz->type, "skin") ||
            strstr(tz->type, "pa")   || strstr(tz->type, "xo"))
            if (temp > max_temp)
                max_temp = temp;
    }

    return max_temp;
}

static bool thermal_check(void)
{
    int temp;

    if (!thermal_limit || thermal_temp <= 0)
        return false;

    temp = read_soc_temp();
    return (temp > 0 && temp > thermal_temp);
}

/* ========================================================================
 * 核心: cpufreq policy notifier
 * ===================================================================== */

static int soc_cpufreq_notify(struct notifier_block *nb,
                               unsigned long action, void *data)
{
    struct cpufreq_policy *policy = data;
    unsigned int desired;
    bool thermal;
    int i;

    if (!enabled)
        return NOTIFY_DONE;

    if (action != CPUFREQ_NOTIFY)
        return NOTIFY_DONE;

    thermal = thermal_check();

    for (i = 0; i < num_clusters; i++) {
        if (clusters[i].cpu != policy->cpu)
            continue;

        /* 选择 cap: 温控优先 */
        if (thermal && thermal_limit > 0) {
            if (!clusters[i].thermal) {
                clusters[i].saved_cap = clusters[i].cap;
                clusters[i].thermal   = true;
            }
            desired = (unsigned int)thermal_limit;
        } else {
            if (clusters[i].thermal) {
                clusters[i].thermal = false;
                clusters[i].cap     = clusters[i].saved_cap;
            }
            desired = clusters[i].cap;
        }

        if (desired == 0 || desired >= clusters[i].hw_max)
            return NOTIFY_DONE;

        /* 拦截并纠正 */
        if (policy->max > desired) {
            pr_debug(DRV_NAME ": cpu%u max %u→%u kHz%s\n",
                     policy->cpu, policy->max, desired,
                     thermal ? " [thermal]" : "");
            policy->max = desired;

            /* 如果当前频率也超了，立刻降下来 */
            if (policy->cur > desired)
                __cpufreq_driver_target(policy, desired,
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
 * 扫描 + 初始应用
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

        /* 温控覆盖 */
        if (thermal_check() && thermal_limit > 0) {
            clusters[idx].saved_cap = clusters[idx].cap;
            clusters[idx].cap       = (unsigned int)thermal_limit;
            clusters[idx].thermal   = true;
        }

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
    if (kstrtoint(buf, 0, &val)) return -EINVAL;
    enabled = !!val;
    return count;
}
static struct kobj_attribute attr_enabled = __ATTR_RW(enabled);

/* freq_limits (rw) — "2000000,2200000,2600000,2800000" */
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
    int vals[MAX_CLUSTERS];
    int n = 0;
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

/* cluster_info (ro) — 调试用 */
static ssize_t cluster_info_show(struct kobject *k, struct kobj_attribute *a,
                                  char *buf)
{
    int pos = 0;
    mutex_lock(&lock);
    for (int i = 0; i < num_clusters; i++)
        pos += scnprintf(buf + pos, PAGE_SIZE - pos,
                         "cluster%d: cpu%u hw_max=%ukHz cap=%ukHz thermal=%d\n",
                         i, clusters[i].cpu, clusters[i].hw_max,
                         clusters[i].cap, clusters[i].thermal);
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

    pr_info(DRV_NAME ": loading — ABK SoC power optimization\n");

    soc_kobj = kobject_create_and_add(DRV_NAME, kernel_kobj);
    if (!soc_kobj) return -ENOMEM;

    ret = sysfs_create_group(soc_kobj, &soc_attr_group);
    if (ret) { kobject_put(soc_kobj); return ret; }

    mutex_lock(&lock);
    soc_scan_and_apply();
    mutex_unlock(&lock);

    ret = cpufreq_register_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    if (ret) {
        pr_err(DRV_NAME ": cpufreq notifier failed (%d)\n", ret);
        sysfs_remove_group(soc_kobj, &soc_attr_group);
        kobject_put(soc_kobj);
        return ret;
    }

    pr_info(DRV_NAME ": ready — %d clusters, notifier active, sysfs at /sys/kernel/%s/\n",
            num_clusters, DRV_NAME);
    return 0;
}

static void __exit abk_soc_opt_exit(void)
{
    cpufreq_unregister_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);

    /* 恢复所有 cluster */
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
MODULE_DESCRIPTION("ABK SoC power optimization — cpufreq cap + thermal throttle + sysfs");
MODULE_VERSION("2.0");
