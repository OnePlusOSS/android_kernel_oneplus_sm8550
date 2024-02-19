// SPDX-License-Identifier: GPL-2.0-only
/*
 * This is based on schedutil governor but modified to work with
 * WALT.
 *
 * Copyright (C) 2016, Intel Corporation
 * Copyright (c) 2020-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kthread.h>
#include <trace/events/power.h>
#include <trace/hooks/sched.h>

#include "walt.h"
#include "trace.h"

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SUGOV_POWER_EFFIENCY)
#include <linux/cpufreq_effiency.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
#include <linux/cpufreq_health.h>
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
#include <../kernel/oplus_cpu/sched/frame_boost/frame_group.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
#include <../kernel/oplus_cpu/sched/eas_opt/oplus_iowait.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
#include <../kernel/oplus_cpu/sched/eas_opt/fake_cap.h>
#endif

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
/* Target load. Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 80
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
#define MAX_CLUSTERS 3
static int init_flag[MAX_CLUSTERS];
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING)
#include <linux/cpufreq_bouncing.h>
#endif

#if IS_ENABLED(CONFIG_OPLUS_OMRG)
#include <linux/oplus_omrg.h>
#endif

struct waltgov_tunables {
	struct gov_attr_set	attr_set;
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		hispeed_load;
	unsigned int		hispeed_freq;
	unsigned int		rtg_boost_freq;
	unsigned int		adaptive_low_freq;
	unsigned int		adaptive_high_freq;
	unsigned int		adaptive_low_freq_kernel;
	unsigned int		adaptive_high_freq_kernel;
	unsigned int		target_load_thresh;
	unsigned int		target_load_shift;
	bool			pl;
	int			boost;
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	spinlock_t		target_loads_lock;
	unsigned int		*target_loads;
	int			ntarget_loads;
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */
};

struct waltgov_policy {
	struct cpufreq_policy	*policy;
	u64			last_ws;
	u64			curr_cycles;
	u64			last_cyc_update_time;
	unsigned long		avg_cap;
	struct waltgov_tunables	*tunables;
	struct list_head	tunables_hook;
	unsigned long		hispeed_util;
	unsigned long		rtg_boost_util;
	unsigned long		max;

	raw_spinlock_t		update_lock;
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	unsigned int		driving_cpu;

	/* The next fields are only needed if fast switch cannot be used: */
	struct	irq_work	irq_work;
	struct	kthread_work	work;
	struct	mutex		work_lock;
	struct	kthread_worker	worker;
	struct task_struct	*thread;

	bool			limits_changed;
	bool			need_freq_update;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
	int newtask_flag;
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	unsigned int		flags;
#endif
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	u64			last_update;
#endif
};

struct waltgov_cpu {
	struct waltgov_callback	cb;
	struct waltgov_policy	*wg_policy;
	unsigned int		cpu;
	struct walt_cpu_load	walt_load;
	unsigned long		util;
	unsigned long		max;
	unsigned int		flags;
	unsigned int		reasons;
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;
#endif
};

DEFINE_PER_CPU(struct waltgov_callback *, waltgov_cb_data);
EXPORT_PER_CPU_SYMBOL_GPL(waltgov_cb_data);
static DEFINE_PER_CPU(struct waltgov_cpu, waltgov_cpu);
static DEFINE_PER_CPU(struct waltgov_tunables *, cached_tunables);

/************************ Governor internals ***********************/

static bool waltgov_should_update_freq(struct waltgov_policy *wg_policy, u64 time)
{
	s64 delta_ns;

	if (unlikely(wg_policy->limits_changed)) {
		wg_policy->limits_changed = false;
		wg_policy->need_freq_update = true;
		return true;
	}

	/*
	 * No need to recalculate next freq for min_rate_limit_us
	 * at least. However we might still decide to further rate
	 * limit once frequency change direction is decided, according
	 * to the separate rate limits.
	 */

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (wg_policy->flags & SCHED_CPUFREQ_DEF_FRAMEBOOST)
		return true;
#endif

	delta_ns = time - wg_policy->last_freq_update_time;
	return delta_ns >= wg_policy->min_rate_limit_ns;
}

static bool waltgov_up_down_rate_limit(struct waltgov_policy *wg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - wg_policy->last_freq_update_time;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	if (wg_policy->flags & SCHED_CPUFREQ_DEF_FRAMEBOOST)
		return false;
#endif

	if (next_freq > wg_policy->next_freq &&
	    delta_ns < wg_policy->up_rate_delay_ns)
		return true;

	if (next_freq < wg_policy->next_freq &&
	    delta_ns < wg_policy->down_rate_delay_ns)
		return true;

	return false;
}

static bool waltgov_update_next_freq(struct waltgov_policy *wg_policy, u64 time,
					unsigned int next_freq,
					unsigned int raw_freq)
{
	if (wg_policy->next_freq == next_freq)
		return false;

	if (waltgov_up_down_rate_limit(wg_policy, time, next_freq)) {
		wg_policy->cached_raw_freq = 0;
		return false;
	}

	wg_policy->cached_raw_freq = raw_freq;
	wg_policy->next_freq = next_freq;
	wg_policy->last_freq_update_time = time;

	return true;
}

static unsigned long freq_to_util(struct waltgov_policy *wg_policy,
				  unsigned int freq)
{
	return mult_frac(wg_policy->max, freq,
			 wg_policy->policy->cpuinfo.max_freq);
}

#define KHZ 1000
static void waltgov_track_cycles(struct waltgov_policy *wg_policy,
				unsigned int prev_freq,
				u64 upto)
{
	u64 delta_ns, cycles;
	u64 next_ws = wg_policy->last_ws + sched_ravg_window;

	upto = min(upto, next_ws);
	/* Track cycles in current window */
	delta_ns = upto - wg_policy->last_cyc_update_time;
	delta_ns *= prev_freq;
	do_div(delta_ns, (NSEC_PER_SEC / KHZ));
	cycles = delta_ns;
	wg_policy->curr_cycles += cycles;
	wg_policy->last_cyc_update_time = upto;
}

static void waltgov_calc_avg_cap(struct waltgov_policy *wg_policy, u64 curr_ws,
				unsigned int prev_freq)
{
	u64 last_ws = wg_policy->last_ws;
	unsigned int avg_freq;

	BUG_ON(curr_ws < last_ws);
	if (curr_ws <= last_ws)
		return;

	/* If we skipped some windows */
	if (curr_ws > (last_ws + sched_ravg_window)) {
		avg_freq = prev_freq;
		/* Reset tracking history */
		wg_policy->last_cyc_update_time = curr_ws;
	} else {
		waltgov_track_cycles(wg_policy, prev_freq, curr_ws);
		avg_freq = wg_policy->curr_cycles;
		avg_freq /= sched_ravg_window / (NSEC_PER_SEC / KHZ);
	}
	wg_policy->avg_cap = freq_to_util(wg_policy, avg_freq);
	wg_policy->curr_cycles = 0;
	wg_policy->last_ws = curr_ws;
}

static void waltgov_fast_switch(struct waltgov_policy *wg_policy, u64 time,
			      unsigned int next_freq)
{
	struct cpufreq_policy *policy = wg_policy->policy;

	waltgov_track_cycles(wg_policy, wg_policy->policy->cur, time);
	cpufreq_driver_fast_switch(policy, next_freq);
}

static void waltgov_deferred_update(struct waltgov_policy *wg_policy, u64 time,
				  unsigned int next_freq)
{
	walt_irq_work_queue(&wg_policy->irq_work);
}

#define TARGET_LOAD 80
static inline unsigned long walt_map_util_freq(unsigned long util,
					struct waltgov_policy *wg_policy,
					unsigned long cap, int cpu)
{
	unsigned long fmax = wg_policy->policy->cpuinfo.max_freq;
	unsigned int shift = wg_policy->tunables->target_load_shift;

	if (util >= wg_policy->tunables->target_load_thresh &&
	    cpu_util_rt(cpu_rq(cpu)) < (cap >> 2))
		return max(
			(fmax + (fmax >> shift)) * util,
			(fmax + (fmax >> 2)) * wg_policy->tunables->target_load_thresh
			)/cap;
	return (fmax + (fmax >> 2)) * util / cap;
}

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
static unsigned int freq_to_targetload(
	struct waltgov_tunables *tunables, unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads - 1 &&
		     freq >= tunables->target_loads[i+1]; i += 2)
		;

	ret = tunables->target_loads[i];
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

unsigned int get_targetload(struct cpufreq_policy *policy)
{
	unsigned int freq = policy->cur;
	unsigned int first_cpu;
	int cluster_id;
	struct waltgov_policy *wg_policy;
	unsigned int target_load = 80;

	first_cpu = cpumask_first(policy->related_cpus);
	cluster_id = topology_physical_package_id(first_cpu);

	if (cluster_id >= MAX_CLUSTERS)
		return target_load;

	if (init_flag[cluster_id] == 0)
		return target_load;

	wg_policy = policy->governor_data;

	if (wg_policy && wg_policy->tunables)
		target_load = freq_to_targetload(wg_policy->tunables, freq);

	return target_load;
}
EXPORT_SYMBOL_GPL(get_targetload);

static unsigned int choose_freq(struct waltgov_policy *wg_policy,
		unsigned int loadadjfreq)
{
	struct cpufreq_policy *policy = wg_policy->policy;
	unsigned int freq = policy->cur;
	unsigned int prevfreq, freqmin, freqmax;
	unsigned int tl;
	int index;

	freqmin = 0;
	freqmax = UINT_MAX;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(wg_policy->tunables, freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		index = cpufreq_frequency_table_target(policy,
						       loadadjfreq / tl,
						       CPUFREQ_RELATION_L);
		freq = policy->freq_table[index].frequency;

		trace_choose_freq(freq, prevfreq, freqmax, freqmin, tl, index);

		if (freq > prevfreq) {
			/* The previous frequency is too low. */
			freqmin = prevfreq;

			if (freq >= freqmax) {
				/*
				 * Find the highest frequency that is less
				 * than freqmax.
				 */
				index = cpufreq_frequency_table_target(
					    policy,
					    freqmax - 1, CPUFREQ_RELATION_H);
				freq = policy->freq_table[index].frequency;

				if (freq == freqmin) {
					/*
					 * The first frequency below freqmax
					 * has already been found to be too
					 * low.  freqmax is the lowest speed
					 * we found that is fast enough.
					 */
					freq = freqmax;
					break;
				}
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq <= freqmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				index = cpufreq_frequency_table_target(
					    policy,
					    freqmin + 1, CPUFREQ_RELATION_L);
				freq = policy->freq_table[index].frequency;

				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				if (freq == freqmax)
					break;
			}
		}

		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SUGOV_POWER_EFFIENCY)
	freq = update_power_effiency_lock(policy, freq, loadadjfreq / tl);
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_POWER_EFFIENCY */

	return freq;
}

void update_util_tl(void *data, unsigned long util, unsigned long freq,
		unsigned long cap, unsigned long *max_util, struct cpufreq_policy *policy,
		bool *need_freq_update)
{
	unsigned int tl = get_targetload(policy);

	*max_util = *max_util * 100 / tl;
}
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

static inline unsigned int get_adaptive_low_freq(struct waltgov_policy *wg_policy)
{
	return(max(wg_policy->tunables->adaptive_low_freq,
		   wg_policy->tunables->adaptive_low_freq_kernel));
}

static inline unsigned int get_adaptive_high_freq(struct waltgov_policy *wg_policy)
{
	return(max(wg_policy->tunables->adaptive_high_freq,
		   wg_policy->tunables->adaptive_high_freq_kernel));
}

static unsigned int get_next_freq(struct waltgov_policy *wg_policy,
				  unsigned long util, unsigned long max,
				  struct waltgov_cpu *wg_cpu, u64 time)
{
	struct cpufreq_policy *policy = wg_policy->policy;
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	unsigned int freq = policy->cpuinfo.max_freq;
	unsigned int prev_freq = freq;
	unsigned int prev_laf = prev_freq * util * 100 / max;
	unsigned int final_freq;

	freq = choose_freq(wg_policy, prev_laf);
	trace_waltgov_next_freq_tl(policy->cpu, util, max, freq, prev_laf, prev_freq);
#else /* !CONFIG_OPLUS_FEATURE_SUGOV_TL */
	unsigned int freq, raw_freq, final_freq;
	struct waltgov_cpu *wg_driv_cpu = &per_cpu(waltgov_cpu, wg_policy->driving_cpu);

	raw_freq = walt_map_util_freq(util, wg_policy, max, wg_driv_cpu->cpu);
	freq = raw_freq;

	if (wg_policy->tunables->adaptive_high_freq) {
		if (raw_freq < get_adaptive_low_freq(wg_policy)) {
			freq = get_adaptive_low_freq(wg_policy);
			wg_driv_cpu->reasons = CPUFREQ_REASON_ADAPTIVE_LOW;
		} else if (raw_freq <= get_adaptive_high_freq(wg_policy)) {
			freq = get_adaptive_high_freq(wg_policy);
			wg_driv_cpu->reasons = CPUFREQ_REASON_ADAPTIVE_HIGH;
		}
	}

	trace_waltgov_next_freq(policy->cpu, util, max, raw_freq, freq, policy->min, policy->max,
				wg_policy->cached_raw_freq, wg_policy->need_freq_update,
				wg_driv_cpu->cpu, wg_driv_cpu->reasons);
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

	if (wg_policy->cached_raw_freq && freq == wg_policy->cached_raw_freq &&
		!wg_policy->need_freq_update)
		return 0;

	wg_policy->need_freq_update = false;

	final_freq = cpufreq_driver_resolve_freq(policy, freq);

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
	cpufreq_health_get_newtask_state(policy, wg_policy->newtask_flag);
#endif

	if (!waltgov_update_next_freq(wg_policy, time, final_freq, freq))
		return 0;

	return final_freq;
}

static unsigned long waltgov_get_util(struct waltgov_cpu *wg_cpu)
{
	struct rq *rq = cpu_rq(wg_cpu->cpu);
	unsigned long max = arch_scale_cpu_capacity(wg_cpu->cpu);
	unsigned long util;

	wg_cpu->max = max;
	wg_cpu->reasons = 0;
	util = cpu_util_freq_walt(wg_cpu->cpu, &wg_cpu->walt_load, &wg_cpu->reasons);
	return uclamp_rq_util_with(rq, util, NULL);
}

#define NL_RATIO 75
#define DEFAULT_HISPEED_LOAD 90
#define DEFAULT_SILVER_RTG_BOOST_FREQ 1000000
#define DEFAULT_GOLD_RTG_BOOST_FREQ 768000
#define DEFAULT_PRIME_RTG_BOOST_FREQ 0
#define DEFAULT_TARGET_LOAD_THRESH 1024
#define DEFAULT_TARGET_LOAD_SHIFT 4
static inline void max_and_reason(unsigned long *cur_util, unsigned long boost_util,
		struct waltgov_cpu *wg_cpu, unsigned int reason)
{
	if (boost_util && boost_util >= *cur_util) {
		*cur_util = boost_util;
		wg_cpu->reasons = reason;
		wg_cpu->wg_policy->driving_cpu = wg_cpu->cpu;
	}
}

static void waltgov_walt_adjust(struct waltgov_cpu *wg_cpu, unsigned long cpu_util,
				unsigned long nl, unsigned long *util,
				unsigned long *max)
{
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	bool is_migration = wg_cpu->flags & WALT_CPUFREQ_IC_MIGRATION;
	bool is_rtg_boost = wg_cpu->walt_load.rtgb_active;
	bool is_hiload;
	bool big_task_rotation = wg_cpu->walt_load.big_task_rotation;
	bool employ_ed_boost = wg_cpu->walt_load.ed_active && sysctl_ed_boost_pct;
	unsigned long pl = wg_cpu->walt_load.pl;

	if (employ_ed_boost) {
		cpu_util = mult_frac(cpu_util, 100 + sysctl_ed_boost_pct, 100);
		max_and_reason(util, cpu_util, wg_cpu, CPUFREQ_REASON_EARLY_DET);
	}

	if (is_rtg_boost)
		max_and_reason(util, wg_policy->rtg_boost_util, wg_cpu, CPUFREQ_REASON_RTG_BOOST);

	is_hiload = (cpu_util >= mult_frac(wg_policy->avg_cap,
					   wg_policy->tunables->hispeed_load,
					   100));

	if (is_hiload && !is_migration)
		max_and_reason(util, wg_policy->hispeed_util, wg_cpu, CPUFREQ_REASON_HISPEED);

	if (is_hiload && nl >= mult_frac(cpu_util, NL_RATIO, 100)) {
		max_and_reason(util, *max, wg_cpu, CPUFREQ_REASON_NWD);
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
		wg_policy->newtask_flag = 1;
	} else {
		wg_policy->newtask_flag = 0;
#endif
	}

	if (wg_policy->tunables->pl) {
		if (sysctl_sched_conservative_pl)
			pl = mult_frac(pl, TARGET_LOAD, 100);
		max_and_reason(util, pl, wg_cpu, CPUFREQ_REASON_PL);
	}

	if (employ_ed_boost)
		wg_cpu->reasons |= CPUFREQ_REASON_EARLY_DET;

	if (big_task_rotation)
		max_and_reason(util, *max, wg_cpu, CPUFREQ_REASON_BTR);
}

static inline unsigned long target_util(struct waltgov_policy *wg_policy,
				  unsigned int freq)
{
	unsigned long util;

	util = freq_to_util(wg_policy, freq);

	if (is_min_cluster_cpu(wg_policy->policy->cpu) &&
		util >= wg_policy->tunables->target_load_thresh)
		util = mult_frac(util, 94, 100);
	else
		util = mult_frac(util, TARGET_LOAD, 100);

	return util;
}

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
/**
 * sugov_iowait_reset() - Reset the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @set_iowait_boost: true if an IO boost has been requested
 *
 * The IO wait boost of a task is disabled after a tick since the last update
 * of a CPU. If a new IO wait boost is requested after more then a tick, then
 * we enable the boost starting from IOWAIT_BOOST_MIN, which improves energy
 * efficiency by ignoring sporadic wakeups from IO.
 */
#define IOWAIT_BOOST_MIN (SCHED_CAPACITY_SCALE / 8)
static bool waltgov_iowait_reset(struct waltgov_cpu *sg_cpu, u64 time,
		bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;
	unsigned int ticks = TICK_NSEC;

	if (sysctl_iowait_reset_ticks)
		ticks = sysctl_iowait_reset_ticks * TICK_NSEC;

	/* Reset boost only if enough ticks has elapsed since last request. */
	if (delta_ns <= ticks)
		return false;

	if (set_iowait_boost)
		sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
	else
		sg_cpu->iowait_boost = 0;

	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}
/**
 * walt_iowait_boost() - Updates the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @flags: SCHED_CPUFREQ_IOWAIT if the task is waking up after an IO wait
 *
 * Each time a task wakes up after an IO operation, the CPU utilization can be
 * boosted to a certain utilization which doubles at each "frequent and
 * successive" wakeup from IO, ranging from IOWAIT_BOOST_MIN to the utilization
 * of the maximum OPP.
 *
 * To keep doubling, an IO boost has to be requested at least once per tick,
 * otherwise we restart from the utilization of the minimum OPP.
 */
static void waltgov_iowait_boost(struct waltgov_cpu *sg_cpu, u64 time,
		unsigned int flags)
{
	bool set_iowait_boost = flags & WALT_CPUFREQ_IOWAIT;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sg_cpu->iowait_boost && waltgov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	/* Boost only tasks waking up after IO */
	if (!set_iowait_boost)
		return;

	/* Ensure boost doubles only one time at each request */
	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	/* Double the boost at each request */
	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost = min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		return;
	}

	/* First wakeup after IO: start with minimum boost */
	sg_cpu->iowait_boost = IOWAIT_BOOST_MIN;
}

/**
 * sugov_iowait_apply() - Apply the IO boost to a CPU.
 * @sg_cpu: the sugov data for the cpu to boost
 * @time: the update time from the caller
 * @util: the utilization to (eventually) boost
 * @max: the maximum value the utilization can be boosted to
 *
 * A CPU running a task which woken up after an IO operation can have its
 * utilization boosted to speed up the completion of those IO operations.
 * The IO boost value is increased each time a task wakes up from IO, in
 * sugov_iowait_apply(), and it's instead decreased by this function,
 * each time an increase has not been requested (!iowait_boost_pending).
 *
 * A CPU which also appears to have been idle for at least one tick has also
 * its IO boost utilization reset.
 *
 * This mechanism is designed to boost high frequently IO waiting tasks, while
 * being more conservative on tasks which does sporadic IO operations.
 */
static unsigned long waltgov_iowait_apply(struct waltgov_cpu *sg_cpu, u64 time,
		unsigned long util, unsigned long max)
{
	struct waltgov_policy *sg_policy = sg_cpu->wg_policy;
	unsigned long boost;

	/* No boost currently required */
	if (!sg_cpu->iowait_boost)
		return util;

	/* Reset boost if the CPU appears to have been idle enough */
	if (waltgov_iowait_reset(sg_cpu, time, false))
		return util;

	if (!sg_cpu->iowait_boost_pending &&
			(!sysctl_iowait_apply_ticks ||
			 (time - sg_policy->last_update > (sysctl_iowait_apply_ticks * TICK_NSEC)))) {
		/*
		 * No boost pending; reduce the boost value.
		 */
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < IOWAIT_BOOST_MIN) {
			sg_cpu->iowait_boost = 0;
			return util;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	/*
	 * @util is already in capacity scale; convert iowait_boost
	 * into the same scale so we can compare.
	 */
	boost = (sg_cpu->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
	return max(boost, util);
}
#endif

static unsigned int waltgov_next_freq_shared(struct waltgov_cpu *wg_cpu, u64 time)
{
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	struct cpufreq_policy *policy = wg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;
	int boost = wg_policy->tunables->boost;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
	unsigned long util_thresh = 0;
	unsigned long util_orig = 0;
	unsigned int avg_nr_running = 1;
	unsigned int count_cpu = 0;
	int cluster_id = topology_physical_package_id(cpumask_first(policy->cpus));
#endif

	for_each_cpu(j, policy->cpus) {
		struct waltgov_cpu *j_wg_cpu = &per_cpu(waltgov_cpu, j);
		unsigned long j_util, j_max, j_nl;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
		struct rq *rq = cpu_rq(j);

		avg_nr_running += rq->nr_running;
		count_cpu++;
#endif
		/*
		 * If the util value for all CPUs in a policy is 0, just using >
		 * will result in a max value of 1. WALT stats can later update
		 * the aggregated util value, causing get_next_freq() to compute
		 * freq = max_freq * 1.25 * (util / max) for nonzero util,
		 * leading to spurious jumps to fmax.
		 */
		j_util = j_wg_cpu->util;
		j_nl = j_wg_cpu->walt_load.nl;
		j_max = j_wg_cpu->max;

#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
		j_util = waltgov_iowait_apply(j_wg_cpu, time, j_util, j_max);
		if (unlikely(eas_opt_debug_enable))
			trace_printk("[eas_opt]: enable_iowait_boost=%d, cpu:%d, max:%d, cpu->util:%d,iowait_util:%d\n",
					sysctl_oplus_iowait_boost_enabled, j_wg_cpu->cpu, j_wg_cpu->max, j_wg_cpu->util, j_util);
#endif
		if (boost) {
			j_util = mult_frac(j_util, boost + 100, 100);
			j_nl = mult_frac(j_nl, boost + 100, 100);
		}

		if (j_util * max >= j_max * util) {
			util = j_util;
			max = j_max;
			wg_policy->driving_cpu = j;
		}

		waltgov_walt_adjust(j_wg_cpu, j_util, j_nl, &util, &max);
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_freq_policy_util(wg_policy->flags, policy->cpus, &util);
#endif
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FAKE_CAP)
	if (eas_opt_enable && (util_thresh_percent[cluster_id] != 100) && count_cpu) {
		//util_thresh = mult_frac(max, util_thresh_percent[cluster_id], 100);
		util_thresh = max * util_thresh_cvt[cluster_id] >> SCHED_CAPACITY_SHIFT;
		avg_nr_running = mult_frac(avg_nr_running, 1, count_cpu);
		util_orig = util;
		//util = avg_nr_running * util * fake_cap_multiple[4] * 1024 / fake_cap_multiple[cluster_id] >>  SCHED_CAPACITY_SHIFT;
		util = (util_thresh < util) ?
			(util_thresh + ((avg_nr_running * (util-util_thresh) * nr_fake_cap_multiple[cluster_id]) >> SCHED_CAPACITY_SHIFT)) : util;
		if (unlikely(eas_opt_debug_enable))
		{
			trace_printk("[eas_opt]: cluster_id: %d, capacity: %d, util_thresh: %d, util_orig: %d, util: %d, avg_nr_running: %d, "
					"fake_cap_multiple: %d,nr_fake_cap_multiple: %d, util_thresh: %d\n",
					cluster_id, max, util_thresh, util_orig, util, avg_nr_running,
					fake_cap_multiple[cluster_id], nr_fake_cap_multiple[cluster_id], util_thresh_percent[cluster_id]);
		}
	}
#endif
	return get_next_freq(wg_policy, util, max, wg_cpu, time);
}

static void waltgov_update_freq(struct waltgov_callback *cb, u64 time,
				unsigned int flags)
{
	struct waltgov_cpu *wg_cpu = container_of(cb, struct waltgov_cpu, cb);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	unsigned long hs_util, rtg_boost_util;
	unsigned int next_f;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	unsigned long irq_flags;
#endif

	if (!wg_policy->tunables->pl && flags & WALT_CPUFREQ_PL)
		return;

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_lock_irqsave(&wg_policy->update_lock, irq_flags);
	wg_cpu->util = waltgov_get_util(wg_cpu);
	wg_cpu->flags = flags;
	wg_policy->flags = flags;
#else
	wg_cpu->util = waltgov_get_util(wg_cpu);
	wg_cpu->flags = flags;
	raw_spin_lock(&wg_policy->update_lock);
#endif

	if (wg_policy->max != wg_cpu->max) {
		wg_policy->max = wg_cpu->max;
		hs_util = target_util(wg_policy,
					wg_policy->tunables->hispeed_freq);
		wg_policy->hispeed_util = hs_util;

		rtg_boost_util = target_util(wg_policy,
				    wg_policy->tunables->rtg_boost_freq);
		wg_policy->rtg_boost_util = rtg_boost_util;
	}
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
	waltgov_iowait_boost(wg_cpu, time, flags);
	wg_cpu->last_update = time;
#endif
	waltgov_calc_avg_cap(wg_policy, wg_cpu->walt_load.ws,
			   wg_policy->policy->cur);

	trace_waltgov_util_update(wg_cpu->cpu, wg_cpu->util, wg_policy->avg_cap,
				wg_cpu->max, wg_cpu->walt_load.nl,
				wg_cpu->walt_load.pl,
				wg_cpu->walt_load.rtgb_active, flags);

	if (waltgov_should_update_freq(wg_policy, time) &&
	    !(flags & WALT_CPUFREQ_CONTINUE)) {
		next_f = waltgov_next_freq_shared(wg_cpu, time);

		if (!next_f)
			goto out;
#if IS_ENABLED(CONFIG_OPLUS_CPUFREQ_IOWAIT_PROTECT)
		wg_policy->last_update = time;
#endif

		if (wg_policy->policy->fast_switch_enabled)
			waltgov_fast_switch(wg_policy, time, next_f);
		else
			waltgov_deferred_update(wg_policy, time, next_f);
	}

out:
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	raw_spin_unlock_irqrestore(&wg_policy->update_lock, irq_flags);
#else
	raw_spin_unlock(&wg_policy->update_lock);
#endif
}

static void waltgov_work(struct kthread_work *work)
{
	struct waltgov_policy *wg_policy = container_of(work, struct waltgov_policy, work);
	unsigned int freq;
	unsigned long flags;

	raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
	freq = wg_policy->next_freq;
	waltgov_track_cycles(wg_policy, wg_policy->policy->cur,
			   walt_sched_clock());
	raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);

	mutex_lock(&wg_policy->work_lock);
	__cpufreq_driver_target(wg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&wg_policy->work_lock);
}

static void waltgov_irq_work(struct irq_work *irq_work)
{
	struct waltgov_policy *wg_policy;

	wg_policy = container_of(irq_work, struct waltgov_policy, irq_work);

	kthread_queue_work(&wg_policy->worker, &wg_policy->work);
}

/************************** sysfs interface ************************/

static inline struct waltgov_tunables *to_waltgov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct waltgov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct waltgov_policy *wg_policy)
{
	mutex_lock(&min_rate_lock);
	wg_policy->min_rate_limit_ns = min(wg_policy->up_rate_delay_ns,
					   wg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->down_rate_limit_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	struct waltgov_policy *wg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		wg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(wg_policy);
	}

	return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	struct waltgov_policy *wg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		wg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(wg_policy);
	}

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->hispeed_load))
		return -EINVAL;

	tunables->hispeed_load = min(100U, tunables->hispeed_load);

	return count;
}

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	unsigned int val;
	struct waltgov_policy *wg_policy;
	unsigned long hs_util;
	unsigned long flags;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->hispeed_freq = val;
	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		hs_util = target_util(wg_policy,
					wg_policy->tunables->hispeed_freq);
		wg_policy->hispeed_util = hs_util;
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
	}

	return count;
}

static ssize_t rtg_boost_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->rtg_boost_freq);
}

static ssize_t rtg_boost_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	unsigned int val;
	struct waltgov_policy *wg_policy;
	unsigned long rtg_boost_util;
	unsigned long flags;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->rtg_boost_freq = val;
	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		rtg_boost_util = target_util(wg_policy,
					  wg_policy->tunables->rtg_boost_freq);
		wg_policy->rtg_boost_util = rtg_boost_util;
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
	}

	return count;
}

static ssize_t pl_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->pl);
}

static ssize_t pl_store(struct gov_attr_set *attr_set, const char *buf,
				   size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	if (kstrtobool(buf, &tunables->pl))
		return -EINVAL;

	return count;
}

static ssize_t boost_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%d\n", tunables->boost);
}

static ssize_t boost_store(struct gov_attr_set *attr_set, const char *buf,
				   size_t count)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	struct waltgov_policy *wg_policy;
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (val < -100 || val > 1000)
		return -EINVAL;

	tunables->boost = val;
	list_for_each_entry(wg_policy, &attr_set->policy_list, tunables_hook) {
		struct rq *rq = cpu_rq(wg_policy->policy->cpu);
		unsigned long flags;

		raw_spin_lock_irqsave(&rq->__lock, flags);
		waltgov_run_callback(rq, WALT_CPUFREQ_BOOST_UPDATE);
		raw_spin_unlock_irqrestore(&rq->__lock, flags);
	}
	return count;
}

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	for (i = 0; i < tunables->ntarget_loads; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret - 1, "%u%s", tunables->target_loads[i],
			i & 0x1 ? ":" : " ");

	snprintf(buf + ret - 1, PAGE_SIZE - ret - 1, "\n");
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;

	return tokenized_data;
err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set, const char *buf,
					size_t count)
{
	int ntokens;
	unsigned int *new_target_loads = NULL;
	unsigned long flags;
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);
	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_target_loads))
		return PTR_ERR(new_target_loads);
	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);

	tunables->target_loads = new_target_loads;
	tunables->ntarget_loads = ntokens;
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);

	return count;
}
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

/**
 * cpufreq_walt_set_adaptive_freq() - set the waltgov adaptive freq for cpu
 * @cpu:               the cpu for which the values should be set
 * @adaptive_low_freq: low freq
 * @adaptive_high_freq:high_freq
 *
 * Configure the adaptive_low/high_freq for the cpu specified. This will impact all
 * cpus governed by the policy (e.g. all cpus in a cluster). The actual value used
 * for adaptive frequencies will be governed by the user space setting for the
 * policy, and this value.
 *
 * Return: 0 if successful, error otherwise
 */
int cpufreq_walt_set_adaptive_freq(unsigned int cpu, unsigned int adaptive_low_freq,
				   unsigned int adaptive_high_freq)
{
	struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;
	struct cpufreq_policy *policy = wg_policy->policy;

	if (!cpu_possible(cpu))
		return -EFAULT;

	if (policy->min <= adaptive_low_freq && policy->max >= adaptive_high_freq) {
		wg_policy->tunables->adaptive_low_freq_kernel = adaptive_low_freq;
		wg_policy->tunables->adaptive_high_freq_kernel = adaptive_high_freq;
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(cpufreq_walt_set_adaptive_freq);

/**
 * cpufreq_walt_get_adaptive_freq() - get the waltgov adaptive freq for cpu
 * @cpu:               the cpu for which the values should be returned
 * @adaptive_low_freq: pointer to write the current kernel adaptive_low_freq value
 * @adaptive_high_freq:pointer to write the current kernel adaptive_high_freq value
 *
 * Get the currently active adaptive_low/high_freq for the cpu specified.
 *
 * Return: 0 if successful, error otherwise
 */
int cpufreq_walt_get_adaptive_freq(unsigned int cpu, unsigned int *adaptive_low_freq,
				   unsigned int *adaptive_high_freq)
{
	struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;

	if (!cpu_possible(cpu))
		return -EFAULT;

	if (adaptive_low_freq && adaptive_high_freq) {
		*adaptive_low_freq = get_adaptive_low_freq(wg_policy);
		*adaptive_high_freq = get_adaptive_high_freq(wg_policy);
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(cpufreq_walt_get_adaptive_freq);

/**
 * cpufreq_walt_reset_adaptive_freq() - reset the waltgov adaptive freq for cpu
 * @cpu:               the cpu for which the values should be set
 *
 * Reset the kernel adaptive_low/high_freq to zero.
 *
 * Return: 0 if successful, error otherwise
 */
int cpufreq_walt_reset_adaptive_freq(unsigned int cpu)
{
	struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);
	struct waltgov_policy *wg_policy = wg_cpu->wg_policy;

	if (!cpu_possible(cpu))
		return -EFAULT;

	wg_policy->tunables->adaptive_low_freq_kernel = 0;
	wg_policy->tunables->adaptive_high_freq_kernel = 0;

	return 0;
}
EXPORT_SYMBOL(cpufreq_walt_reset_adaptive_freq);

#define WALTGOV_ATTR_RW(_name)						\
static struct governor_attr _name =					\
__ATTR(_name, 0644, show_##_name, store_##_name)			\

#define show_attr(name)							\
static ssize_t show_##name(struct gov_attr_set *attr_set, char *buf)	\
{									\
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);	\
	return scnprintf(buf, PAGE_SIZE, "%lu\n", tunables->name);	\
}									\

#define store_attr(name)						\
static ssize_t store_##name(struct gov_attr_set *attr_set,		\
				const char *buf, size_t count)		\
{									\
	struct waltgov_tunables *tunables = to_waltgov_tunables(attr_set);	\
										\
	if (kstrtouint(buf, 10, &tunables->name))			\
		return -EINVAL;						\
									\
	return count;							\
}									\

show_attr(adaptive_low_freq);
store_attr(adaptive_low_freq);
show_attr(adaptive_high_freq);
store_attr(adaptive_high_freq);
show_attr(target_load_thresh);
store_attr(target_load_thresh);
show_attr(target_load_shift);
store_attr(target_load_shift);

static struct governor_attr hispeed_load = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq = __ATTR_RW(hispeed_freq);
static struct governor_attr rtg_boost_freq = __ATTR_RW(rtg_boost_freq);
static struct governor_attr pl = __ATTR_RW(pl);
static struct governor_attr boost = __ATTR_RW(boost);
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
static struct governor_attr target_loads =
	__ATTR(target_loads, 0664, target_loads_show, target_loads_store);
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */
WALTGOV_ATTR_RW(adaptive_low_freq);
WALTGOV_ATTR_RW(adaptive_high_freq);
WALTGOV_ATTR_RW(target_load_thresh);
WALTGOV_ATTR_RW(target_load_shift);

static struct attribute *waltgov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&hispeed_load.attr,
	&hispeed_freq.attr,
	&rtg_boost_freq.attr,
	&pl.attr,
	&boost.attr,
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	&target_loads.attr,
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */
	&adaptive_low_freq.attr,
	&adaptive_high_freq.attr,
	&target_load_thresh.attr,
	&target_load_shift.attr,
	NULL
};

static struct kobj_type waltgov_tunables_ktype = {
	.default_attrs	= waltgov_attributes,
	.sysfs_ops	= &governor_sysfs_ops,
};

/********************** cpufreq governor interface *********************/

static struct cpufreq_governor walt_gov;

static struct waltgov_policy *waltgov_policy_alloc(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy;

	wg_policy = kzalloc(sizeof(*wg_policy), GFP_KERNEL);
	if (!wg_policy)
		return NULL;

	wg_policy->policy = policy;
	raw_spin_lock_init(&wg_policy->update_lock);
	return wg_policy;
}

static void waltgov_policy_free(struct waltgov_policy *wg_policy)
{
	kfree(wg_policy);
}

static int waltgov_kthread_create(struct waltgov_policy *wg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO / 2 };
	struct cpufreq_policy *policy = wg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&wg_policy->work, waltgov_work);
	kthread_init_worker(&wg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &wg_policy->worker,
				"waltgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create waltgov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	wg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&wg_policy->irq_work, waltgov_irq_work);
	mutex_init(&wg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void waltgov_kthread_stop(struct waltgov_policy *wg_policy)
{
	/* kthread only required for slow path */
	if (wg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&wg_policy->worker);
	kthread_stop(wg_policy->thread);
	mutex_destroy(&wg_policy->work_lock);
}

static void waltgov_tunables_save(struct cpufreq_policy *policy,
		struct waltgov_tunables *tunables)
{
	int cpu;
	struct waltgov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!cached) {
		cached = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!cached)
			return;

		for_each_cpu(cpu, policy->related_cpus)
			per_cpu(cached_tunables, cpu) = cached;
	}

	cached->pl = tunables->pl;
	cached->hispeed_load = tunables->hispeed_load;
	cached->rtg_boost_freq = tunables->rtg_boost_freq;
	cached->hispeed_freq = tunables->hispeed_freq;
	cached->up_rate_limit_us = tunables->up_rate_limit_us;
	cached->down_rate_limit_us = tunables->down_rate_limit_us;
	cached->boost = tunables->boost;
	cached->adaptive_low_freq = tunables->adaptive_low_freq;
	cached->adaptive_high_freq = tunables->adaptive_high_freq;
	cached->adaptive_low_freq_kernel = tunables->adaptive_low_freq_kernel;
	cached->adaptive_high_freq_kernel = tunables->adaptive_high_freq_kernel;
	cached->target_load_thresh = tunables->target_load_thresh;
	cached->target_load_shift = tunables->target_load_shift;
}

static void waltgov_tunables_restore(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	struct waltgov_tunables *tunables = wg_policy->tunables;
	struct waltgov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!cached)
		return;

	tunables->pl = cached->pl;
	tunables->hispeed_load = cached->hispeed_load;
	tunables->rtg_boost_freq = cached->rtg_boost_freq;
	tunables->hispeed_freq = cached->hispeed_freq;
	tunables->up_rate_limit_us = cached->up_rate_limit_us;
	tunables->down_rate_limit_us = cached->down_rate_limit_us;
	tunables->boost	= cached->boost;
	tunables->adaptive_low_freq = cached->adaptive_low_freq;
	tunables->adaptive_high_freq = cached->adaptive_high_freq;
	tunables->adaptive_low_freq_kernel = cached->adaptive_low_freq_kernel;
	tunables->adaptive_high_freq_kernel = cached->adaptive_high_freq_kernel;
	tunables->target_load_thresh = cached->target_load_thresh;
	tunables->target_load_shift = cached->target_load_shift;
}

static int waltgov_init(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy;
	struct waltgov_tunables *tunables;
	int ret = 0;
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	unsigned int first_cpu;
	int cluster_id;
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

#if IS_ENABLED(CONFIG_OPLUS_OMRG)
        omrg_cpufreq_register(policy);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_OCH)
	if(cpufreq_health_register(policy))
		pr_err("cpufreq health init failed!\n");
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_SUGOV_POWER_EFFIENCY)
        frequence_opp_init(policy);
#endif

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_GKI_CPUFREQ_BOUNCING)
        cb_stuff_init(policy);
#endif

	if (policy->fast_switch_possible && !policy->fast_switch_enabled)
		BUG_ON(1);

	wg_policy = waltgov_policy_alloc(policy);
	if (!wg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = waltgov_kthread_create(wg_policy);
	if (ret)
		goto free_wg_policy;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	gov_attr_set_init(&tunables->attr_set, &wg_policy->tunables_hook);
	tunables->hispeed_load = DEFAULT_HISPEED_LOAD;
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	tunables->target_loads = default_target_loads;
	tunables->ntarget_loads = ARRAY_SIZE(default_target_loads);
	spin_lock_init(&tunables->target_loads_lock);
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */
	tunables->target_load_thresh = DEFAULT_TARGET_LOAD_THRESH;
	tunables->target_load_shift = DEFAULT_TARGET_LOAD_SHIFT;

	if (is_min_cluster_cpu(policy->cpu))
		tunables->rtg_boost_freq = DEFAULT_SILVER_RTG_BOOST_FREQ;
	else if (is_max_cluster_cpu(policy->cpu))
		tunables->rtg_boost_freq = DEFAULT_PRIME_RTG_BOOST_FREQ;
	else
		tunables->rtg_boost_freq = DEFAULT_GOLD_RTG_BOOST_FREQ;

	policy->governor_data = wg_policy;
	wg_policy->tunables = tunables;
	waltgov_tunables_restore(policy);

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &waltgov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   walt_gov.name);
	if (ret)
		goto fail;

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	first_cpu = cpumask_first(policy->related_cpus);
	cluster_id = topology_physical_package_id(first_cpu);
	if (cluster_id < MAX_CLUSTERS)
		init_flag[cluster_id] = 1;
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	kfree(tunables);
stop_kthread:
	waltgov_kthread_stop(wg_policy);
free_wg_policy:
	waltgov_policy_free(wg_policy);
disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void waltgov_exit(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	struct waltgov_tunables *tunables = wg_policy->tunables;
	unsigned int count;
#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	unsigned int first_cpu;
	int cluster_id;

	first_cpu = cpumask_first(policy->related_cpus);
	cluster_id = topology_physical_package_id(first_cpu);
	if (cluster_id < MAX_CLUSTERS)
		init_flag[cluster_id] = 0;
#endif /* CONFIG_OPLUS_FEATURE_SUGOV_TL */

	count = gov_attr_set_put(&tunables->attr_set, &wg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		waltgov_tunables_save(policy, tunables);
		kfree(tunables);
	}

	waltgov_kthread_stop(wg_policy);
	waltgov_policy_free(wg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int waltgov_start(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned int cpu;

	wg_policy->up_rate_delay_ns =
		wg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	wg_policy->down_rate_delay_ns =
		wg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(wg_policy);
	wg_policy->last_freq_update_time	= 0;
	wg_policy->next_freq			= 0;
	wg_policy->limits_changed		= false;
	wg_policy->need_freq_update		= false;
	wg_policy->cached_raw_freq		= 0;
#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	wg_policy->flags           		= 0;
#endif

	for_each_cpu(cpu, policy->cpus) {
		struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);

		memset(wg_cpu, 0, sizeof(*wg_cpu));
		wg_cpu->cpu			= cpu;
		wg_cpu->wg_policy		= wg_policy;
	}

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	register_trace_android_vh_map_util_freq_new(update_util_tl, NULL);
#endif

	for_each_cpu(cpu, policy->cpus) {
		struct waltgov_cpu *wg_cpu = &per_cpu(waltgov_cpu, cpu);

		waltgov_add_callback(cpu, &wg_cpu->cb, waltgov_update_freq);
	}

#if IS_ENABLED(CONFIG_OPLUS_FEATURE_FRAME_BOOST)
	fbg_add_update_freq_hook(waltgov_run_callback);
#endif
	return 0;
}

static void waltgov_stop(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		waltgov_remove_callback(cpu);

#ifdef CONFIG_OPLUS_FEATURE_SUGOV_TL
	unregister_trace_android_vh_map_util_freq_new(update_util_tl, NULL);
#endif

	synchronize_rcu();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&wg_policy->irq_work);
		kthread_cancel_work_sync(&wg_policy->work);
	}
}

static void waltgov_limits(struct cpufreq_policy *policy)
{
	struct waltgov_policy *wg_policy = policy->governor_data;
	unsigned long flags, now;
	unsigned int freq, final_freq;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&wg_policy->work_lock);
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		waltgov_track_cycles(wg_policy, wg_policy->policy->cur,
				   walt_sched_clock());
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&wg_policy->work_lock);
	} else {
		raw_spin_lock_irqsave(&wg_policy->update_lock, flags);
		freq = policy->cur;
		now = walt_sched_clock();

		/*
		 * cpufreq_driver_resolve_freq() has a clamp, so we do not need
		 * to do any sort of additional validation here.
		 */
		final_freq = cpufreq_driver_resolve_freq(policy, freq);

		if (waltgov_update_next_freq(wg_policy, now, final_freq,
			final_freq)) {
			waltgov_fast_switch(wg_policy, now, final_freq);
		}
		raw_spin_unlock_irqrestore(&wg_policy->update_lock, flags);
	}

	wg_policy->limits_changed = true;
}

static struct cpufreq_governor walt_gov = {
	.name			= "walt",
	.init			= waltgov_init,
	.exit			= waltgov_exit,
	.start			= waltgov_start,
	.stop			= waltgov_stop,
	.limits			= waltgov_limits,
	.owner			= THIS_MODULE,
};

int waltgov_register(void)
{
	return cpufreq_register_governor(&walt_gov);
}
