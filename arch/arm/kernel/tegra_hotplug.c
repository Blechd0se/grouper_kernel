/*
 * Copyright (c) 2013, Francisco Franco <franciscofranco.1990@gmail.com>. 
 *
 * Small algorithm changes for more performance
 * Copyright (c) 2013, Alexander Christ <alex.christ@hotmail.de>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Simple no bullshit hot[un]plug driver for SMP
 */

/*
 * TODO   - Hotplug driver makes static decisions (thread migraten could be expansive)
 *        - Add Thermal Throttle Driver
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/earlysuspend.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/hotplug.h>
 
#define DEFAULT_FIRST_LEVEL	70
#define LOAD_BALANCER		10
#define HIGH_LOAD_COUNTER	20
#define SAMPLING_RATE_MS	500

struct cpu_stats
{
	unsigned int total_cpus;
	unsigned int default_first_level;
	unsigned int default_load_balancer;

	/* For the three hot-plug-able Cores */
	unsigned int counter[2];
};

struct cpu_load_data {
        u64 prev_cpu_idle;
        u64 prev_cpu_wall;
};

static DEFINE_PER_CPU(struct cpu_load_data, cpuload);

static struct cpu_stats stats;
static struct workqueue_struct *wq;
static struct delayed_work decide_hotplug;
static struct workqueue_struct *pm_wq;
static struct work_struct resume;
static struct work_struct suspend;

static unsigned long queue_sampling;

static inline int get_cpu_load(unsigned int cpu)
{
	struct cpu_load_data *pcpu = &per_cpu(cpuload, cpu);
	struct cpufreq_policy policy;
	u64 cur_wall_time, cur_idle_time;
	unsigned int idle_time, wall_time;
	unsigned int cur_load;

	cpufreq_get_policy(&policy, cpu);

	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time, true);

	wall_time = (unsigned int) (cur_wall_time - pcpu->prev_cpu_wall);
	pcpu->prev_cpu_wall = cur_wall_time;

	idle_time = (unsigned int) (cur_idle_time - pcpu->prev_cpu_idle);
	pcpu->prev_cpu_idle = cur_idle_time;

	if (unlikely(!wall_time || wall_time < idle_time))
		return 0;

	cur_load = 100 * (wall_time - idle_time) / wall_time;

	return (cur_load * policy.cur) / policy.max;
}

/*
 * Calculates the load for a given cpu
 */ 

static void calculate_load_for_cpu(int cpu) 
{
	struct cpufreq_policy policy;

	for_each_online_cpu(cpu) {
		cpufreq_get_policy(&policy, cpu);
		/*  
		 * We are above our threshold, so update our counter for cpu.
		 * Consider this only, if we are on our max frequency
		 */
		if (get_cpu_load(cpu) >= stats.default_first_level
			&& likely(stats.counter[cpu] < HIGH_LOAD_COUNTER)
			&& cpufreq_quick_get(cpu) == policy.max) {
				stats.counter[cpu] += 2;
		}

		else {
			if (stats.counter[cpu] > 0)
				stats.counter[cpu]--;
		}
		
		if (stats.counter[cpu] > 0 && 
			get_cpu_load(cpu) <= stats.default_load_balancer)
				stats.counter[cpu]--;

		/* Reset CPU */
		if (cpu)
			break;
	}	

}

/**
 * Simple load based decision algorithm to determ
 * how many cores should be on- or offlined
 */

static void __ref decide_hotplug_func(struct work_struct *work)
{
	static unsigned long last_change_time;
	int i, j;

	/* Do load calculation for each cpu counter */

	for (i = 0, j = 2; i < 2; i++, j++) {
		calculate_load_for_cpu(i);

		if (stats.counter[i] >= 10) {
			if (!cpu_online(j)) {
				printk("[Hot-Plug]: CPU%u ready for onlining\n", j);
				cpu_up(j);
				last_change_time = ktime_to_ms(ktime_get());
			}
		}
		else {
			/* Prevent fast on-/offlining */ 
			if ((ktime_to_ms(ktime_get()) + (SAMPLING_RATE_MS * 8)) > last_change_time) {
				calculate_load_for_cpu(i);

				if (stats.counter[i] > 0 && cpu_online(j)) {
						printk("[Hot-Plug]: CPU%u ready for offlining\n", j);
						
						if (get_cpu_load(j) > get_cpu_load(j+1) 
								&& cpu_online(j+1))
							cpu_down(j+1);
						else if (get_cpu_load(j) > get_cpu_load(j-1) 
								&& cpu_online(j-1) && j-1 != 1)
							cpu_down(j-1);
						else
							cpu_down(j); 

						last_change_time = ktime_to_ms(ktime_get());
				}
			}
		}
	}
	
	/* Make a dedicated work_queue */
	queue_delayed_work(wq, &decide_hotplug, queue_sampling);
}

static void suspend_func(struct work_struct *work)
{	 
	int cpu;

	/* cancel the hotplug work when the screen is off and flush the WQ */
	flush_workqueue(wq);
	cancel_delayed_work_sync(&decide_hotplug);
	cancel_work_sync(&resume);

	pr_info("Early Suspend stopping Hotplug work...\n");
    
	for_each_online_cpu(cpu) 
		if (cpu)
			cpu_down(cpu);
}

static void __ref resume_func(struct work_struct *work)
{
	int cpu;

	/* online all cores when the screen goes online */
	for_each_possible_cpu(cpu) {
		if (cpu) 
			cpu_up(cpu);
	}

	cancel_work_sync(&suspend);

	/* Resetting Counters */
	stats.counter[0] = 0;
	stats.counter[1] = 0;

	pr_info("Late Resume starting Hotplug work...\n");
	queue_delayed_work(wq, &decide_hotplug, HZ);
}

static void grouper_hotplug_early_suspend(struct early_suspend *handler)
{   
	queue_work(pm_wq, &suspend);
}

static void grouper_hotplug_late_resume(struct early_suspend *handler)
{  
	queue_work(pm_wq, &resume);
}

static struct early_suspend grouper_hotplug_suspend =
{
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = grouper_hotplug_early_suspend,
	.resume = grouper_hotplug_late_resume,
};

int __init grouper_hotplug_init(void)
{
	pr_info("Grouper Hotplug driver started.\n");
    
	/* init everything here */
	stats.total_cpus = num_present_cpus();
	stats.default_first_level = DEFAULT_FIRST_LEVEL;
	stats.default_load_balancer = LOAD_BALANCER;
	queue_sampling = msecs_to_jiffies(SAMPLING_RATE_MS);
	
	/* Resetting Counters */
	stats.counter[0] = 0;
	stats.counter[1] = 0;

	wq = alloc_ordered_workqueue("grouper_hotplug_workqueue", 0);
    
	if (!wq)
		return -ENOMEM;

	pm_wq = alloc_workqueue("grouper_pm_workqueue", 0, 1);

	if (!pm_wq)
		return -ENOMEM;
    
	INIT_DELAYED_WORK(&decide_hotplug, decide_hotplug_func);
	INIT_WORK(&resume, resume_func);
	INIT_WORK(&suspend, suspend_func);
	queue_delayed_work(wq, &decide_hotplug, queue_sampling);

	register_early_suspend(&grouper_hotplug_suspend);
    
	return 0;
}
late_initcall(grouper_hotplug_init);


