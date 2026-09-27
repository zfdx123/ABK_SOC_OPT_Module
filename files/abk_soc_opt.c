/*
 * abk_soc_opt.c — ABK SoC 功耗优化内核模块 v2.5
 *
 * 内核态 cpufreq policy notifier，拦截用户空间频率修改并强制锁定上限。
 *
 * v2.5 用户态门控 (userspace-gated):
 *   核心变化: 内核态默认【完全不生效】，必须由用户态显式 arm 才开始工作；
 *   用户态一旦消失，看门狗自动收起全部约束。
 *
 *   动机: 避免"内核态在跑但用户态没了"造成的半生效状态 —— 那种状态下
 *   内核会单方面压频率、离线核心，而没有任何东西负责恢复或协调。
 *
 *   接口:
 *     echo <pid> > /sys/kernel/abk_soc_opt/arm   启用，以该 pid 为持有者
 *     echo 0     > /sys/kernel/abk_soc_opt/arm   主动收起
 *     重复写同一 pid 即续期心跳（开销为零）
 *     cat /sys/kernel/abk_soc_opt/active         内核态当前是否真的在施加约束
 *     cat /sys/kernel/abk_soc_opt/armed_pid      当前持有者 pid
 *
 *   失效条件（任一满足即自动 disarm 并恢复中性状态）:
 *     - 持有者进程退出（通过 get_pid_task 判断，不受 pid 复用影响）
 *     - arm_timeout_ms 内未续期心跳（默认 30000ms，设 0 关闭看门狗）
 *     - 用户态主动写 0
 *     - enabled=0（模块参数或 sysfs）
 *
 *   副作用（正面）: 扫描改为懒执行 —— 只有 arm 时才扫 cluster，
 *   那时 cpufreq 驱动必然已就绪，"本模块 init 早于 qcom-cpufreq-hw"的
 *   builtin 时序问题从根上消失，不再需要靠重试去补。
 *
 * v2.4 编译修复 (v2.3 编成 .ko 时 modpost 报 undefined):
 *   - device_offline()/device_online() 在 v6.1 里没有 EXPORT_SYMBOL。
 *     改用 EXPORT_SYMBOL_GPL 导出的 add_cpu()/remove_cpu() —— 内核源码
 *     明确写着 "Other subsystems should use remove_cpu()/add_cpu() instead"。
 *     实现上 remove_cpu() 内部就是 lock_device_hotplug()+device_offline()，
 *     行为完全一致，只是由它自己持锁。
 *   - cpumask_var_t 换成内嵌 cpumask_t。alloc/free_cpumask_var 的 EXPORT
 *     只在 CONFIG_CPUMASK_OFFSTACK=y 时才存在（lib/cpumask.c 整段被
 *     #ifdef 包住），换构建配置就会重现同类报错。内嵌写法还顺带消除了
 *     "扫描提前失败导致 cpumask 泄漏"的问题。
 *   设备实测确认: CONFIG_HOTPLUG_CPU=y / CONFIG_SMP=y，
 *   __ksymtab_remove_cpu、__ksymtab_add_cpu 均存在。
 *
 * v2.3 关键修复:
 *   [致命] 初始化时序: 原实现在 device_initcall 级别扫描 cluster，而
 *          qcom-cpufreq-hw 是独立模块、更晚 probe，导致 cpufreq_cpu_get()
 *          对每个 CPU 都返回 NULL，num_clusters 被记成 0，模块从此永久失效
 *          （sysfs 节点存在且接受写入，但什么都不生效，且 builtin 无法重载）。
 *          现在：扫描失败会重试，并在首个 CPUFREQ_CREATE_POLICY 时补扫；
 *          另加 scan sysfs 属性供用户态强制重扫。
 *   [高]   freq 上限改用 freq_qos 约束（cpufreq.max）+ policy->max 双保险。
 *          只写 policy->max 会被 qcom-cpufreq-hw 自身的 limits 管理覆盖。
 *   [高]   notifier 重入保护: 原实现持 mutex 时调用 __cpufreq_driver_target()，
 *          该调用会再次触发 CPUFREQ_POLICY_NOTIFIER，递归抢同一把 mutex。
 *          现在不在 notifier 里调用驱动接口，且加了重入计数。
 *   [中]   clusters_online 计数错误: 只统计 cap>0 的簇，导致全部 cap 为 0
 *          时计数器恒为 0，"至少保留一个簇在线"的保护形同虚设。
 *   [中]   restore_all_clusters() 只遍历 num_clusters，扫描提前失败时
 *          已分配的 cpumask 会泄漏、已离线的核不会恢复。
 *   [低]   新增 num_clusters / scan sysfs 属性，便于用户态判断内核态是否
 *          真的在工作（原实现只能靠 cluster_info 为空来猜）。
 *
 * v2.2:
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
#include <linux/pm_qos.h>
#include <linux/atomic.h>

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

    /* 内嵌 cpumask，不用 cpumask_var_t。
     *
     * cpumask_var_t + alloc_cpumask_var_node()/free_cpumask_var() 只在
     * CONFIG_CPUMASK_OFFSTACK=y 时才是真正的导出函数（lib/cpumask.c 里
     * 整段被 #ifdef 包住）。本机该配置为 n，虽然宏会退化成 nop 从而"碰巧
     * 能编译"，但一旦换构建配置打开 OFFSTACK 就会立刻报
     *   modpost: "free_cpumask_var" ... undefined!
     * 而 MAX_CLUSTERS 只有 4，内嵌 4 个 cpumask 的代价可以忽略。
     * 内嵌写法还能顺带消除原来"扫描提前失败导致 cpumask 泄漏"的问题。 */
    cpumask_t     cpus;

    bool          offlined;

    /* freq_qos 约束: 把上限钉在 cpufreq 的 QoS 层。
     * 只写 policy->max 会被 qcom-cpufreq-hw 自己的 limits 机制覆盖,
     * QoS 约束则会在每次 policy 重算时被框架重新合并进来。 */
    struct freq_qos_request qos_max;
    bool          qos_active;
};

static struct soc_cluster clusters[MAX_CLUSTERS];
static int                num_clusters;
static int                clusters_online;  /* count of non-offlined clusters */
static struct mutex       lock;

/* notifier 重入保护。notifier 回调里可能间接再次触发
 * CPUFREQ_POLICY_NOTIFIER，递归抢 lock 会死锁。 */
static atomic_t           nb_depth = ATOMIC_INIT(0);

/* 扫描只做一次，失败则重试 */
static atomic_t           scan_done = ATOMIC_INIT(0);

/* ========================================================================
 * 用户态持有 (arming) —— 只有用户态在跑，内核态才生效
 * =====================================================================
 *
 * 设计目标:
 *   1. 内核态默认【完全不生效】。加载/内建进来只是注册接口，不扫 cluster、
 *      不装 QoS、不碰任何频率。
 *   2. 用户态写完 arm 之后才生效；用户态一死（崩溃/被杀/超时未续期），
 *      看门狗自动收起全部约束，回到中性状态。
 *   3. 这样"内核态在跑但用户态没了"造成的半生效状态在结构上不可能出现。
 *
 * 持有方式用 PID + 心跳超时，不用 fd 持有:
 *   fd 持有（char 设备 + release 回调）语义上更精确，但需要额外的设备节点
 *   和 SELinux 标签，跨机型风险高；PID + 心跳只需 sysfs，且用户态用
 *   AppOpt 的现有主循环续期即可，零额外依赖。
 *
 *   arm 语义:  echo <pid> > /sys/kernel/abk_soc_opt/arm   启用并以该 pid 为持有者
 *              echo 0     > /sys/kernel/abk_soc_opt/arm   主动收起
 */
static pid_t  armed_pid;
static unsigned long armed_last_jiffies;
static bool   enforced;              /* 当前是否已施加约束 */

/* 心跳超时(毫秒)。0 = 不启用看门狗，纯手动 arm/disarm。
 * 用户态需在此时间内至少续期一次，否则内核自动 disarm。 */
static unsigned int arm_timeout_ms = 30000;
module_param(arm_timeout_ms, uint, 0644);
MODULE_PARM_DESC(arm_timeout_ms,
    "Userspace heartbeat timeout in ms (0=no watchdog, default 30000)");

/* polling */
static struct delayed_work poll_work;
/* 扫描重试 */
static struct delayed_work scan_retry_work;
#define SCAN_RETRY_MAX   60
#define SCAN_RETRY_MS    500
static int scan_retry_count;

/* 持有者看门狗 */
static struct delayed_work arm_watchdog_work;
#define ARM_WATCHDOG_PERIOD_MS  2000
/* 重新 arm 的节流：防抖，避免用户态高频写 arm 反复触发扫描 */
#define ARM_REARM_THROTTLE_MS   1000

/* 前向声明。
 * 注意 arm_watchdog_start：它被 arm_enable() 调用，但定义在其后，
 * 漏了声明会报 "call to undeclared function"（C99 起不再允许隐式声明）。 */
static void soc_scan_and_apply(void);
static void restore_all_clusters(void);
static void poll_start(void);
static void poll_stop(void);
static void arm_watchdog_start(void);
static void arm_enable(void);
static void arm_disable(void);

/* 进程是否还活着。用 get_pid_task 拿引用再立刻归还，
 * 避免 pid 复用导致的误判。 */
static bool arm_owner_alive(void)
{
    struct task_struct *t;
    bool alive;

    if (!armed_pid)
        return false;

    rcu_read_lock();
    t = get_pid_task(find_vpid(armed_pid), PIDTYPE_PID);
    rcu_read_unlock();
    if (!t)
        return false;

    alive = (READ_ONCE(t->__state) != TASK_DEAD);
    put_task_struct(t);
    return alive;
}

/* 收起全部约束，回到中性状态 */
static void arm_disable(void)
{
    if (!enforced)
        return;

    poll_stop();
    cancel_delayed_work_sync(&scan_retry_work);
    restore_all_clusters();      /* 摘 QoS、恢复频率上限、把离线的核放回来 */
    enforced = false;
    pr_info(DRV_NAME ": disarmed, all constraints released\n");
}

/* 施加约束。可重入：已生效时直接返回。 */
static void arm_enable(void)
{
    if (enforced)
        return;

    soc_scan_and_apply();

    if (num_clusters == 0) {
        /* cpufreq 还没就绪，启动重试；真正生效交给 scan_retry_cb */
        pr_info(DRV_NAME ": armed but cpufreq not ready, retrying\n");
        scan_retry_count = 0;
        schedule_delayed_work(&scan_retry_work, msecs_to_jiffies(SCAN_RETRY_MS));
        return;
    }

    atomic_set(&scan_done, 1);
    poll_start();
    enforced = true;
    arm_watchdog_start();
    pr_info(DRV_NAME ": armed by pid %d, %d clusters enforcing\n",
            armed_pid, num_clusters);
}

/* 看门狗：持有者死了或超时未续期就自动收起 */
static void arm_watchdog_cb(struct work_struct *work)
{
    bool timeout = false;

    if (arm_timeout_ms > 0) {
        unsigned long deadline =
            armed_last_jiffies + msecs_to_jiffies(arm_timeout_ms) / 2;
        timeout = time_after(jiffies, deadline);
    }

    if (!armed_pid || timeout || !arm_owner_alive()) {
        if (armed_pid) {
            pr_warn(DRV_NAME ": owner gone (pid=%d timeout=%d alive=%d), disarming\n",
                    armed_pid, (int)timeout, (int)arm_owner_alive());
            armed_pid = 0;
            arm_disable();
        }
        return;   /* 已收起，不再重排 */
    }

    schedule_delayed_work(&arm_watchdog_work,
                          msecs_to_jiffies(ARM_WATCHDOG_PERIOD_MS));
}

static void arm_watchdog_start(void)
{
    schedule_delayed_work(&arm_watchdog_work,
                          msecs_to_jiffies(ARM_WATCHDOG_PERIOD_MS));
}

/* 统一的开关入口：sysfs 和模块参数都走这里，保证语义一致 */
static void soc_set_enabled(int val)
{
    enabled = !!val;

    if (enabled && armed_pid) {
        arm_enable();
    } else if (!enabled) {
        arm_disable();
    }
}

/* ========================================================================
 * 核心下线/上线
 * ===================================================================== */

/* 前向声明：下线/上线核心时要同步增删 QoS 约束 */
static void cluster_set_qos(struct soc_cluster *c, unsigned int cap_khz);
static int  cluster_add_qos(struct soc_cluster *c, unsigned int cap_khz);

/* 下线一个 cluster 的全部核心。首次调用时若 CPU 不存在则静默跳过。
 *
 * 用 add_cpu()/remove_cpu() 而不是直接调 device_online()/device_offline():
 * 后者在 v6.1 里没有 EXPORT_SYMBOL，编成 .ko 时 modpost 会报
 *   "device_offline" [drivers/abk_soc_opt/abk_soc_opt.ko] undefined!
 *   "device_online"  [drivers/abk_soc_opt/abk_soc_opt.ko] undefined!
 * 而 add_cpu()/remove_cpu() 是 EXPORT_SYMBOL_GPL 导出的，且 kernel/cpu.c
 * 里明确注明 "Other subsystems should use remove_cpu()/add_cpu() instead"。
 *
 * 实现上 remove_cpu() 内部就是 lock_device_hotplug() + device_offline()，
 * 所以 sysfs 的 device->offline 状态同样会同步，行为与原来一致，
 * 只是锁由它自己负责 —— 调用方不需要也不能再持 cpu_hotplug_lock。
 */
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

    /* 先撤掉 QoS 约束：核心即将离线，留着约束没有意义，
     * 而且卸载时再 remove 会触发 WARN（request 已不活跃）。 */
    cluster_set_qos(c, 0);

    for_each_cpu(cpu, &c->cpus) {
        /* CPU0 不能被下线，内核会拒绝；显式跳过避免刷警告 */
        if (cpu == 0)
            continue;
        if (cpu_online(cpu))
            remove_cpu(cpu);
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

    for_each_cpu(cpu, &c->cpus) {
        if (!cpu_online(cpu))
            add_cpu(cpu);
    }
    c->offlined = false;
    clusters_online++;

    /* 核心回来了，重新装上约束 */
    if (c->cap > 0)
        cluster_add_qos(c, c->cap);

    pr_info(DRV_NAME ": cluster cpu%u onlined\n", c->first_cpu);
}

/* ========================================================================
 * 频率强制
 * ===================================================================== */

/* 把一个 cluster 的频率上限同步到 freq_qos 约束。
 *
 * 关于取值: FREQ_QOS_MAX 的 request 值是"允许的最大频率(kHz)"。
 * 不设约束时不能传 FREQ_QOS_MAX_DEFAULT_VALUE(S32_MAX) —— 它在
 * freq_qos_update_request() 里会被 `new_value < 0` 判为非法并返回 -EINVAL
 * (S32_MAX 转成 s32 是 -1)。所以约束的存在性用 add/remove 管理，
 * 而不是用哨兵值。 */
static void cluster_set_qos(struct soc_cluster *c, unsigned int cap_khz)
{
    int ret;

    if (cap_khz == 0) {
        if (c->qos_active) {
            freq_qos_remove_request(&c->qos_max);
            c->qos_active = false;
        }
        return;
    }

    if (!c->qos_active) {
        /* 首次建立约束需要 policy，失败就下次再试 */
        cluster_add_qos(c, cap_khz);
        return;
    }

    ret = freq_qos_update_request(&c->qos_max, (s32)cap_khz);
    if (ret < 0)
        pr_warn(DRV_NAME ": cpu%u freq_qos update to %u failed (%d)\n",
                c->first_cpu, cap_khz, ret);
}

static int cluster_add_qos(struct soc_cluster *c, unsigned int cap_khz)
{
    struct cpufreq_policy *policy;
    int ret;

    if (c->qos_active || cap_khz == 0)
        return 0;

    policy = cpufreq_cpu_get(c->first_cpu);
    if (!policy)
        return -ENODEV;

    /* FREQ_QOS_MAX 的 request 值就是"允许的最大频率" */
    ret = freq_qos_add_request(&policy->constraints, &c->qos_max,
                               FREQ_QOS_MAX, (s32)cap_khz);
    cpufreq_cpu_put(policy);

    if (ret < 0) {
        pr_warn(DRV_NAME ": cpu%u freq_qos_add_request(%u) failed (%d)\n",
                c->first_cpu, cap_khz, ret);
        return ret;
    }

    c->qos_active = true;
    return 0;
}

/* 对一个 cluster 的 policy 应用 cap。
 *
 * 注意调用上下文：
 *   - 从 workqueue 调用: 自己拿 policy->rwsem 读锁后再改。
 *   - 从 cpufreq notifier 调用: notifier 本身就跑在 policy->rwsem 写锁下,
 *     此时【不能】再取读锁（会死锁），由调用者通过 in_notifier 告知。
 * 本函数不调用 __cpufreq_driver_target()，避免在 notifier 上下文里
 * 再次触发 notifier 递归。
 */
static void cluster_apply_cap(struct soc_cluster *c, bool in_notifier)
{
    struct cpufreq_policy *policy;

    if (c->offlined || c->cap == 0)
        return;

    policy = cpufreq_cpu_get(c->first_cpu);
    if (!policy)
        return;

    /* QoS 约束是主力: 它在框架层被合并, 不受驱动自身 limits 覆盖影响 */
    cluster_set_qos(c, c->cap);

    if (!in_notifier)
        down_read(&policy->rwsem);

    if (policy->max > c->cap) {
        pr_debug(DRV_NAME ": cpu%u max %u→%u kHz\n",
                 c->first_cpu, policy->max, c->cap);
        policy->max = c->cap;
    }

    if (!in_notifier)
        up_read(&policy->rwsem);

    cpufreq_cpu_put(policy);
}

/* 对所有 cluster 应用 cap（workqueue / sysfs 上下文） */
static void enforce_all(void)
{
    int i;

    mutex_lock(&lock);
    for (i = 0; i < num_clusters; i++) {
        if (!clusters[i].offlined)
            cluster_apply_cap(&clusters[i], false);
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

    /* 未生效时内核态必须完全不干预。这是"只有用户态在跑内核态才生效"
     * 的关键闸门：没有持有者就一路返回，不碰 policy、不改频率。 */
    if (!enabled || !enforced)
        return NOTIFY_DONE;

    if (action == CPUFREQ_CREATE_POLICY ||
        action == CPUFREQ_REMOVE_POLICY) {
        /* 已 arm 但在等 cpufreq 就绪时，借 CREATE_POLICY 触发补扫。
         * （v2.5 改为懒扫描后这种情况只会在 arm 后驱动尚未就绪时出现） */
        if (action == CPUFREQ_CREATE_POLICY && atomic_read(&scan_done) == 0)
            schedule_delayed_work(&scan_retry_work, 0);
        return NOTIFY_DONE;
    }

    /* 重入保护: notifier 里若间接再次进入本函数, 直接返回,
     * 否则会在 mutex_lock 上自死锁。 */
    if (atomic_inc_return(&nb_depth) > 1) {
        atomic_dec(&nb_depth);
        return NOTIFY_DONE;
    }

    mutex_lock(&lock);
    for (i = 0; i < num_clusters; i++) {
        if (clusters[i].first_cpu != policy->cpu)
            continue;
        if (!clusters[i].offlined)
            cluster_apply_cap(&clusters[i], true);
        break;
    }
    mutex_unlock(&lock);

    atomic_dec(&nb_depth);
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
    if (num_clusters > 0)
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

/* 扫描重试: arm 之后 cpufreq 驱动可能还没就绪（builtin 场景）。
 * 用 delayed_work 而不是在 notifier 里直接调 cpufreq_cpu_get(),
 * 避免在持有 policy->rwsem 时再去拿 cpufreq_driver_lock。
 *
 * 注意: 每次重试前都要检查是否仍处于 arm 状态 —— 持有者可能在重试期间
 * 就退出了，这时必须立刻放弃，不能"复活"约束。 */
static void scan_retry_cb(struct work_struct *work)
{
    if (enforced || !armed_pid) {
        /* 已生效，或持有者已消失：收工 */
        return;
    }

    if (scan_retry_count >= SCAN_RETRY_MAX) {
        pr_warn(DRV_NAME ": giving up cluster scan after %d retries "
                         "(cpufreq driver never became ready)\n", scan_retry_count);
        armed_pid = 0;
        arm_disable();
        return;
    }

    scan_retry_count++;
    soc_scan_and_apply();

    if (num_clusters > 0) {
        pr_info(DRV_NAME ": cluster scan succeeded on retry %d\n", scan_retry_count);
        /* 交给统一入口置位 enforced / 起轮询 / 起看门狗，
         * 避免这里漏掉状态变更（曾导致 enforced 永远起不来）。 */
        arm_enable();
    } else {
        schedule_delayed_work(&scan_retry_work, msecs_to_jiffies(SCAN_RETRY_MS));
    }
}

/* 清空扫描状态：必须先摘掉所有 freq_qos 请求，再清零结构体。
 *
 * 顺序很关键 —— freq_qos 请求登记在 policy->constraints 的链表里，
 * 如果先 memset 掉 clusters[]，链表里就留下指向已清零内存的悬挂节点，
 * 之后 policy 重算 / 模块卸载时移除它会踩到野指针。
 * 只有 qos_active 为 true 的请求才是"已登记"的，remove 对未登记请求会 WARN。
 *
 * cpumask 是内嵌的，随 memset 一起清零，无需单独释放。 */
static void scan_state_reset(void)
{
    int i;

    for (i = 0; i < MAX_CLUSTERS; i++) {
        if (clusters[i].qos_active) {
            freq_qos_remove_request(&clusters[i].qos_max);
            clusters[i].qos_active = false;
        }
    }
    memset(clusters, 0, sizeof(clusters));
    clusters_online = 0;
    num_clusters = 0;
}

static void soc_scan_and_apply(void)
{
    struct cpufreq_policy *policy;
    int idx = 0;
    int cpu;

    mutex_lock(&lock);

    scan_state_reset();

    /* 填充 cluster 表。cpufreq_cpu_get/put 会拿 cpufreq_driver_lock，
     * 但不会回调 CPUFREQ_POLICY_NOTIFIER，所以持 lock 调用是安全的。 */
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

        /* 拷贝 CPU 掩码（内嵌 cpumask，已在 scan_state_reset 里清零） */
        cpumask_copy(&clusters[idx].cpus, policy->related_cpus);

        clusters[idx].first_cpu = cpu;
        clusters[idx].hw_max    = policy->cpuinfo.max_freq;
        clusters[idx].cap       = 0;
        clusters[idx].offlined  = false;

        if (idx < num_freq_limits)
            clusters[idx].cap = (unsigned int)freq_limits[idx];
        /* cap < 0 → treat as 0 (offline) */
        if ((int)clusters[idx].cap < 0)
            clusters[idx].cap = 0;

        clusters_online++;

        if (clusters[idx].cap > 0) {
            pr_info(DRV_NAME ": cluster%d cpu%u hw=%u cap=%u kHz\n",
                    idx, cpu, clusters[idx].hw_max, clusters[idx].cap);
            if (policy->max > clusters[idx].cap)
                policy->max = clusters[idx].cap;
        }

        idx++;
        cpufreq_cpu_put(policy);
    }

    num_clusters = idx;

    if (num_clusters == 0) {
        mutex_unlock(&lock);
        pr_warn(DRV_NAME ": no cpufreq policy found yet "
                         "(driver not ready), will retry\n");
        return;
    }

    /* 建 QoS 约束 + offline cap==0 的簇。
     * 这两步都会走到 cpufreq_cpu_get() / device_offline()，必须放在锁外，
     * 否则和 notifier 路径形成 lock→cpufreq_driver_lock 的嵌套。 */
    for (idx = 0; idx < num_clusters; idx++) {
        if (clusters[idx].cap > 0)
            cluster_add_qos(&clusters[idx], clusters[idx].cap);
        else
            cluster_offline(&clusters[idx]);
    }

    mutex_unlock(&lock);

    /* 建完约束后统一强制一遍 */
    enforce_all();

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
    /* 走统一入口：enabled=1 也需要已有持有者才会真正生效，
     * 避免"只翻开关但没有用户态"这种半生效状态。 */
    soc_set_enabled(val);
    return count;
}
static struct kobj_attribute attr_enabled = __ATTR_RW(enabled);

/* arm (wo) — 用户态持有/续期。这是内核态生效的唯一前提。
 *
 *   echo <pid> > arm    以该 pid 为持有者启用（重复写同一 pid 即续期心跳）
 *   echo 0     > arm    主动收起
 */
static ssize_t arm_store(struct kobject *k, struct kobj_attribute *a,
                          const char *buf, size_t count)
{
    pid_t pid;
    if (kstrtoint(buf, 0, &pid)) return -EINVAL;

    if (pid <= 0) {
        armed_pid = 0;
        arm_disable();
        pr_info(DRV_NAME ": disarmed by userspace\n");
        return count;
    }

    if (armed_pid != pid) {
        /* 换持有者：先收起旧的，再按新 pid 启用 */
        arm_disable();
        armed_pid = 0;
    }

    armed_last_jiffies = jiffies;

    if (!enabled) {
        /* 模块参数被显式关掉了，只记录持有者不生效 */
        pr_info(DRV_NAME ": armed by pid %d but enabled=0, not enforcing\n", pid);
        return count;
    }

    if (!enforced) {
        armed_pid = pid;
        arm_enable();       /* 内部会置位 enforced、起轮询、起看门狗 */
    } else {
        /* 已在生效中，视作心跳续期，开销为零 */
        armed_pid = pid;
    }

    return count;
}
static struct kobj_attribute attr_arm = __ATTR_WO(arm);

/* armed_pid (ro) — 用户态可读回当前持有者，便于自检 */
static ssize_t armed_pid_show(struct kobject *k, struct kobj_attribute *a,
                               char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", armed_pid);
}
static struct kobj_attribute attr_armed_pid = __ATTR_RO(armed_pid);

/* active (ro) — 内核态当前是否真的在施加约束。
 * 这是判断"内核态是否生效"的权威标志，比看 cluster_info 是否为空更直接。 */
static ssize_t active_show(struct kobject *k, struct kobj_attribute *a,
                            char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", enforced ? 1 : 0);
}
static struct kobj_attribute attr_active = __ATTR_RO(active);

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

    /* 内核态一次都没扫到 cluster 时，这里必须明确报错，而不是"接受写入但
     * 什么都不做"。旧实现静默吞掉写入，用户态完全看不出内核态是死的
     * （num_clusters==0 时下面那个 for 直接空转）。 */
    if (num_clusters == 0) {
        pr_warn(DRV_NAME ": freq_limits write ignored - no cluster detected yet\n");
        return -ENODEV;
    }

    mutex_lock(&lock);
    for (int i = 0; i < n && i < num_clusters; i++) {
        unsigned int new_cap = (vals[i] < 0) ? 0 : (unsigned int)vals[i];
        bool was_online = !clusters[i].offlined;

        clusters[i].cap = new_cap;

        if (new_cap == 0 && was_online) {
            cluster_offline(&clusters[i]);
        } else if (new_cap > 0 && clusters[i].offlined) {
            cluster_online(&clusters[i]);
            cluster_apply_cap(&clusters[i], false);
        } else if (new_cap > 0 && was_online) {
            cluster_apply_cap(&clusters[i], false);
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

    poll_stop();
    poll_ms = val;
    poll_start();

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

/* num_clusters (ro) — 让用户态能一眼判断内核态是否真的在工作 */
static ssize_t num_clusters_show(struct kobject *k, struct kobj_attribute *a,
                                  char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%d\n", num_clusters);
}
static struct kobj_attribute attr_num_clusters = __ATTR_RO(num_clusters);

/* scan (wo) — 强制重扫。用于 builtin 场景下启动时序问题的手动补救：
 *   echo 1 > /sys/kernel/abk_soc_opt/scan
 */
static ssize_t scan_store(struct kobject *k, struct kobj_attribute *a,
                           const char *buf, size_t count)
{
    int val;
    if (kstrtoint(buf, 0, &val)) return -EINVAL;
    if (val) {
        atomic_set(&scan_done, 0);
        scan_retry_count = 0;
        cancel_delayed_work_sync(&scan_retry_work);
        soc_scan_and_apply();
        if (num_clusters > 0) {
            atomic_set(&scan_done, 1);
            pr_info(DRV_NAME ": manual rescan ok, %d clusters\n", num_clusters);
        } else {
            schedule_delayed_work(&scan_retry_work, 0);
        }
    }
    return count;
}
static struct kobj_attribute attr_scan = __ATTR_WO(scan);

static struct attribute *soc_attrs[] = {
    &attr_enabled.attr,
    &attr_arm.attr,
    &attr_armed_pid.attr,
    &attr_active.attr,
    &attr_freq_limits.attr,
    &attr_poll_ms.attr,
    &attr_cluster_info.attr,
    &attr_num_clusters.attr,
    &attr_scan.attr,
    NULL,
};
static const struct attribute_group soc_attr_group = { .attrs = soc_attrs };

/* ========================================================================
 * 模块生命周期
 * ===================================================================== */

static void restore_all_clusters(void)
{
    int i;

    /* 遍历 MAX_CLUSTERS 而不是 num_clusters:
     * 扫描可能提前失败使 num_clusters=0，但前面的槽位仍持有 QoS 约束，
     * 只按 num_clusters 遍历会漏掉它们。
     * cpumask 是内嵌的，不需要释放。 */
    for (i = 0; i < MAX_CLUSTERS; i++) {
        if (clusters[i].qos_active) {
            freq_qos_remove_request(&clusters[i].qos_max);
            clusters[i].qos_active = false;
        }

        if (clusters[i].offlined)
            cluster_online(&clusters[i]);

        /* 恢复被我们压下去的频率上限 */
        if (clusters[i].hw_max > 0) {
            struct cpufreq_policy *policy =
                cpufreq_cpu_get(clusters[i].first_cpu);
            if (policy) {
                if (policy->max < clusters[i].hw_max)
                    policy->max = clusters[i].hw_max;
                cpufreq_cpu_put(policy);
            }
        }
    }
    memset(clusters, 0, sizeof(clusters));
    clusters_online = 0;
    num_clusters = 0;
}

static int __init abk_soc_opt_init(void)
{
    int ret;

    pr_info(DRV_NAME ": loading v2.5 (userspace-gated)\n");

    mutex_init(&lock);

    soc_kobj = kobject_create_and_add(DRV_NAME, kernel_kobj);
    if (!soc_kobj) return -ENOMEM;

    ret = sysfs_create_group(soc_kobj, &soc_attr_group);
    if (ret) { kobject_put(soc_kobj); return ret; }

    INIT_DELAYED_WORK(&poll_work, poll_timer_cb);
    INIT_DELAYED_WORK(&scan_retry_work, scan_retry_cb);
    INIT_DELAYED_WORK(&arm_watchdog_work, arm_watchdog_cb);

    /* 只注册 notifier，不注册回调动作之外的东西。
     * notifier 在未 arm 时会在第一行直接返回。 */
    ret = cpufreq_register_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    if (ret) {
        pr_err(DRV_NAME ": cpufreq notifier failed (%d)\n", ret);
        sysfs_remove_group(soc_kobj, &soc_attr_group);
        kobject_put(soc_kobj);
        return ret;
    }

    /* 关键：这里【不做】任何 cluster 扫描、不装 QoS、不改频率。
     *
     * 之所以可以放心不扫: v2.5 的扫描改成了懒执行 —— 用户态 arm 时才扫，
     * 那时 cpufreq 驱动必然已就绪，builtin 的 initcall 时序问题就此消失。
     *
     * 默认状态: 未 arm，内核态完全不生效。 */
    pr_info(DRV_NAME ": idle — waiting for userspace arming. "
                     "echo <pid> > /sys/kernel/%s/arm to activate\n", DRV_NAME);
    return 0;
}

static void __exit abk_soc_opt_exit(void)
{
    poll_stop();
    cancel_delayed_work_sync(&scan_retry_work);
    cancel_delayed_work_sync(&arm_watchdog_work);
    arm_disable();

    cpufreq_unregister_notifier(&soc_nb, CPUFREQ_POLICY_NOTIFIER);
    sysfs_remove_group(soc_kobj, &soc_attr_group);
    kobject_put(soc_kobj);

    pr_info(DRV_NAME ": unloaded — all cores restored\n");
}

module_init(abk_soc_opt_init);
module_exit(abk_soc_opt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("AppOpt");
MODULE_DESCRIPTION("ABK SoC power optimization — userspace-gated cpufreq cap + core offlining");
MODULE_VERSION("2.5");
