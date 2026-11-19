/*
 * AP_HAL_Zephyr diagnostics monitor (CONFIG_AP_HAL_ZEPHYR_MONITOR):
 * a thread above the ArduPilot main thread, so it still reports when main
 * spins. One printk line per period:
 *
 *   mon <s>: tk=<scheduler ticks> task=<index in tasks.txt, -1 between
 *   tasks, -3 fast loop> loops=<n> max=<us> avg=<us> long=<n> init=<0|1>
 *   heap=<free KiB> ins=<mock samples> rp rx=<B> tx=<B> st=<stalled sends>
 *   dr=<dropped RX B> no=<mailbox notifies> ki=<kicks>
 *   cpu <thread>=<% of the period> ...
 *
 * and every AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY lines the scheduler task table
 * (the @SYS/tasks.txt text). loops/max/avg/long and the table cover the time
 * since the previous table.
 *
 * Without logging (HAL_LOGGING_ENABLED 0) the vehicle never runs
 * AP_Scheduler::update_logging(), which allocates the per-task statistics
 * and resets perf_info: the monitor does both. The task-info array is sized
 * for 255 tasks (perf_info bounds-checks every index; ~6 KiB).
 */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_AP_HAL_ZEPHYR_MONITOR)

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <AP_Common/ExpandingString.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if HAL_ZEPHYR_RPMSG_SERIAL_ENABLED
#include "ipc/ap_rpmsg_serial.h"
#endif
#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS
#include "MockINS.h"
#endif

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint8_t MON_MAX_THREADS = 24;

struct mon_thread {
    const struct k_thread *tid;
    uint64_t cycles;
};

struct mon_state {
    mon_thread prev[MON_MAX_THREADS];
    mon_thread now[MON_MAX_THREADS];
    uint8_t n_now;
    uint64_t total;
};

mon_state mon;

void mon_collect(const struct k_thread *cthread, void *user)
{
    auto *st = static_cast<mon_state *>(user);
    k_thread_runtime_stats_t rt;

    if (st->n_now >= MON_MAX_THREADS ||
        k_thread_runtime_stats_get(const_cast<k_tid_t>(cthread), &rt) != 0) {
        return;
    }
    st->now[st->n_now].tid = cthread;
    st->now[st->n_now].cycles = rt.execution_cycles;
    st->n_now++;
}

uint64_t mon_prev_cycles(const struct k_thread *tid)
{
    for (auto &p : mon.prev) {
        if (p.tid == tid) {
            return p.cycles;
        }
    }
    return 0;
}

void mon_cpu_line(void)
{
    char line[256];
    size_t pos = 0;
    uint64_t delta[MON_MAX_THREADS];
    uint64_t sum = 0;

    mon.n_now = 0;
    k_thread_foreach_unlocked(mon_collect, &mon);
    for (uint8_t i = 0; i < mon.n_now; i++) {
        const uint64_t prev = mon_prev_cycles(mon.now[i].tid);
        delta[i] = mon.now[i].cycles >= prev ? mon.now[i].cycles - prev : 0;
        sum += delta[i];
    }
    pos += snprintk(line + pos, sizeof(line) - pos, "cpu");
    for (uint8_t i = 0; i < mon.n_now && pos < sizeof(line) - 24; i++) {
        const char *name = k_thread_name_get(const_cast<k_tid_t>(mon.now[i].tid));
        const unsigned pct = sum ? (unsigned)((delta[i] * 100U + sum / 2U) / sum) : 0U;
        pos += snprintk(line + pos, sizeof(line) - pos, " %.10s=%u",
                        (name && name[0]) ? name : "?", pct);
    }
    printk("%s\n", line);
    memcpy(mon.prev, mon.now, sizeof(mon.now[0]) * mon.n_now);
    for (uint8_t i = mon.n_now; i < MON_MAX_THREADS; i++) {
        mon.prev[i].tid = nullptr;
    }
}

void mon_tasks(void)
{
    AP_Scheduler *sched = AP_Scheduler::get_singleton();
    ExpandingString str;

    if (sched == nullptr) {
        return;
    }
    sched->task_info(str);
    // reset after printing: the next table and the loop figures cover the
    // next interval (update_logging() does this when logging is built)
    sched->perf_info.reset();
    if (str.has_failed_allocation() || str.get_length() == 0) {
        return;
    }
    // printk in chunks: a long single %s may be cut by the formatter
    const char *p = str.get_string();
    for (uint32_t off = 0; off < str.get_length(); off += 128) {
        printk("%.128s", p + off);
    }
    printk("\n");
}

void mon_fn(void *, void *, void *)
{
    uint32_t n = 0;
    bool tasks_enabled = false;

    for (;;) {
        k_sleep(K_MSEC(CONFIG_AP_HAL_ZEPHYR_MONITOR_PERIOD_MS));
        n++;
        AP_Scheduler *sched = AP_Scheduler::get_singleton();
        const bool init = hal.scheduler->is_system_initialized();
        uint32_t ticks = 0, loops = 0, max_us = 0, avg_us = 0, nlong = 0;
        if (sched != nullptr) {
            ticks = sched->ticks32();
            loops = sched->perf_info.get_num_loops();
            max_us = sched->perf_info.get_max_time();
            avg_us = sched->perf_info.get_avg_time();
            nlong = sched->perf_info.get_num_long_running();
        }
        uint32_t ins = 0;
#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS
        ins = ap_mock_ins_samples;
#endif
        printk("mon %u: tk=%u task=%d loops=%u max=%u avg=%u long=%u init=%d heap=%uk ins=%u",
               (unsigned)(k_uptime_get() / 1000), (unsigned)ticks,
               (int)hal.util->persistent_data.scheduler_task, (unsigned)loops,
               (unsigned)max_us, (unsigned)avg_us, (unsigned)nlong, init ? 1 : 0,
               (unsigned)(hal.util->available_memory() / 1024U), (unsigned)ins);
#if HAL_ZEPHYR_RPMSG_SERIAL_ENABLED
        struct ap_rpmsg_serial_stats rp;
        ap_rpmsg_serial_stats_get(&rp);
        printk(" rp rx=%u tx=%u st=%u dr=%u no=%u ki=%u", (unsigned)rp.rx_bytes,
               (unsigned)rp.tx_bytes, (unsigned)rp.tx_stalls, (unsigned)rp.rx_dropped,
               (unsigned)rp.notifies, (unsigned)rp.kicks);
#endif
        printk("\n");
        mon_cpu_line();
        if (init && CONFIG_AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY > 0) {
            if (!tasks_enabled) {
                ExpandingString tmp;
                sched->task_info(tmp); // first call sets RECORD_TASK_INFO
                if (!sched->perf_info.has_task_info()) {
                    sched->perf_info.allocate_task_info(255);
                }
                sched->perf_info.reset();
                tasks_enabled = true;
            } else if (n % CONFIG_AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY == 0) {
                mon_tasks();
            }
        }
    }
}

K_THREAD_STACK_DEFINE(mon_stack, 4096);
struct k_thread mon_thread_data;

} // namespace

// called from HAL_Zephyr::run() (a SYS_INIT here would be dropped: nothing
// else references this object in the static HAL library)
void ap_hal_zephyr_monitor_start(void)
{
    // preemptible prio 1: above ap_timer (2), rpmsg (5) and main (8)
    k_thread_create(&mon_thread_data, mon_stack, K_THREAD_STACK_SIZEOF(mon_stack), mon_fn,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), K_FP_REGS, K_NO_WAIT);
    k_thread_name_set(&mon_thread_data, "ap_mon");
}

#endif
