/*
 * AP_HAL_Zephyr diagnostics monitor (CONFIG_AP_HAL_ZEPHYR_MONITOR).
 *
 * A thread just above the ArduPilot main thread (priority 7, main is 8): it
 * still reports when main spins, and it preempts nothing but main and the
 * background threads. It does NOT report when a thread above it spins (timer,
 * rpmsg, a driver thread).
 *
 * The monitor thread only READS ArduPilot state. Everything that writes it
 * or allocates (enabling the per-task statistics, building the task table
 * with its float formatting, perf_info.reset()) is done by the MAIN thread
 * in ap_hal_zephyr_monitor_main_service(), called from the HAL main loop,
 * when the monitor has set a request flag; the monitor then prints the table
 * from a static buffer in small chunks, sleeping in between. A stuck main
 * loop therefore gets no task table, only the status lines.
 *
 * Every AP_HAL_ZEPHYR_MONITOR_PRINT_EVERY periods, two printk lines:
 *
 *   mon <s>: tk=<scheduler ticks> task=<index in tasks.txt, -1 between
 *   tasks, -3 fast loop> loops=<n> max=<us> avg=<us> long=<n> init=<0|1>
 *   heap=<free KiB> ins=<mock samples> dl=<late delay_microseconds()
 *   sleeps>/<worst lateness us> rp rx=<B> tx=<B> st=<stalled sends>
 *   dr=<dropped RX B> no=<mailbox notifies> ki=<kicks>
 *   cpu <thread>=<% since the previous cpu line> ...
 *
 * every AP_HAL_ZEPHYR_MONITOR_STACKS_EVERY printed lines the never-used
 * bytes of every thread stack and of the exception stacks (needs
 * CONFIG_INIT_STACKS + CONFIG_THREAD_STACK_INFO):
 *
 *   stk <thread>=<free B> ... svc=<free B> irq=<free B>
 *
 * (on Cortex-R the interrupt handlers run on the SVC stack), and every
 * AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY printed lines the scheduler task table
 * (the @SYS/tasks.txt text). loops/max/avg/long and the table cover the time
 * since the previous table.
 *
 * Without logging (HAL_LOGGING_ENABLED 0) the vehicle never runs
 * AP_Scheduler::update_logging(), which allocates the per-task statistics
 * and resets perf_info: the main-thread service does both. The task-info
 * array is sized for 255 tasks (perf_info bounds-checks every index; ~6 KiB).
 *
 * Console budget: the two status lines are about 300 B, the stack line about
 * 250 B, the task table about 5 KiB. With the defaults (1 s, stacks every
 * 10, table every 30 lines) that is ~500 B/s: the AI-64's 60 KiB RAM console
 * (a ring: the oldest text is overwritten, the boot log first) holds about
 * the last 2 minutes. AP_HAL_ZEPHYR_MONITOR_PRINT_EVERY=10 stretches that to
 * ~20 minutes (the table then comes every 300 s).
 */
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_AP_HAL_ZEPHYR_MONITOR)

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include <AP_Common/ExpandingString.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if HAL_ZEPHYR_RPMSG_SERIAL_ENABLED
#include "ipc/ap_rpmsg_serial.h"
#endif
#ifdef CONFIG_AP_HAL_ZEPHYR_MOCK_SENSORS
#include "MockINS.h"
#endif

#ifndef CONFIG_AP_HAL_ZEPHYR_MONITOR_PRINT_EVERY
#define CONFIG_AP_HAL_ZEPHYR_MONITOR_PRINT_EVERY 1
#endif
#ifndef CONFIG_AP_HAL_ZEPHYR_MONITOR_STACKS_EVERY
#define CONFIG_AP_HAL_ZEPHYR_MONITOR_STACKS_EVERY 10
#endif

#if defined(CONFIG_INIT_STACKS) && defined(CONFIG_THREAD_STACK_INFO) && \
    CONFIG_AP_HAL_ZEPHYR_MONITOR_STACKS_EVERY > 0
#define MON_HAVE_STACKS 1
#else
#define MON_HAVE_STACKS 0
#endif

#if CONFIG_AP_HAL_ZEPHYR_DELAY_SLEEP_MIN_US > 0
extern uint32_t ap_hal_zephyr_delay_late_count;
extern uint32_t ap_hal_zephyr_delay_late_max_us;
#endif

#if MON_HAVE_STACKS && defined(CONFIG_CPU_AARCH32_CORTEX_R)
extern "C" {
K_KERNEL_STACK_ARRAY_DECLARE(z_arm_svc_stack, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ARMV7_SVC_STACK_SIZE);
K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);
}
#endif

extern const AP_HAL::HAL &hal;

namespace {

constexpr uint8_t MON_MAX_THREADS = 24;
constexpr int MON_PRIO = 7;          // just above main (8)
constexpr size_t MON_CHUNK = 128;    // table bytes per printk
constexpr size_t MON_TABLE_MAX = 8192;

struct mon_thread {
    const struct k_thread *tid;
    uint64_t cycles;
};

struct mon_state {
    mon_thread prev[MON_MAX_THREADS];
    mon_thread now[MON_MAX_THREADS];
    uint8_t n_now;
};

mon_state mon;

/*
 * monitor -> main requests, main -> monitor table. The monitor sets a
 * request only while table_ready is 0 and reads the buffer only while it is
 * 1; main writes the buffer only while serving a request: never both.
 */
enum : int {
    REQ_NONE = 0,
    REQ_ENABLE,     // enable + allocate the per-task statistics, reset
    REQ_TABLE,      // build the task table, then reset
};
atomic_t mon_request = ATOMIC_INIT(REQ_NONE);
atomic_t mon_table_ready = ATOMIC_INIT(0);
bool mon_tasks_enabled;       // written by main, read by the monitor
char mon_table[MON_TABLE_MAX];
size_t mon_table_len;

// k_thread_foreach() holds the kernel thread-list lock: only collect here
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

const char *mon_name(const struct k_thread *tid)
{
    const char *name = k_thread_name_get(const_cast<k_tid_t>(tid));
    return (name && name[0]) ? name : "?";
}

void mon_cpu_line(void)
{
    char line[256];
    size_t pos = 0;
    uint64_t delta[MON_MAX_THREADS];
    uint64_t sum = 0;

    mon.n_now = 0;
    k_thread_foreach(mon_collect, &mon);
    for (uint8_t i = 0; i < mon.n_now; i++) {
        const uint64_t prev = mon_prev_cycles(mon.now[i].tid);
        delta[i] = mon.now[i].cycles >= prev ? mon.now[i].cycles - prev : 0;
        sum += delta[i];
    }
    pos += snprintk(line + pos, sizeof(line) - pos, "cpu");
    for (uint8_t i = 0; i < mon.n_now && pos < sizeof(line) - 24; i++) {
        const unsigned pct = sum ? (unsigned)((delta[i] * 100U + sum / 2U) / sum) : 0U;
        pos += snprintk(line + pos, sizeof(line) - pos, " %.10s=%u", mon_name(mon.now[i].tid), pct);
    }
    printk("%s\n", line);
    memcpy(mon.prev, mon.now, sizeof(mon.now[0]) * mon.n_now);
    for (uint8_t i = mon.n_now; i < MON_MAX_THREADS; i++) {
        mon.prev[i].tid = nullptr;
    }
}

#if MON_HAVE_STACKS
#if defined(CONFIG_CPU_AARCH32_CORTEX_R)
// bytes never written since z_arm_init_stacks() filled the stack with 0xAA
// (stacks grow down: the untouched part is at the low end)
size_t mon_unused(const uint8_t *buf, size_t size)
{
    size_t n = 0;
    while (n < size && buf[n] == 0xAAU) {
        n++;
    }
    return n;
}
#endif

// uses the thread list of the last mon_cpu_line(); the scans (a 64 KiB main
// stack is ~64 KiB of byte compares) run here, outside the thread-list lock
void mon_stack_line(void)
{
    char line[320];
    size_t pos = 0;

    pos += snprintk(line + pos, sizeof(line) - pos, "stk");
    for (uint8_t i = 0; i < mon.n_now && pos < sizeof(line) - 48; i++) {
        size_t unused = 0;
        if (k_thread_stack_space_get(mon.now[i].tid, &unused) != 0) {
            continue;
        }
        pos += snprintk(line + pos, sizeof(line) - pos, " %.10s=%u", mon_name(mon.now[i].tid),
                        (unsigned)unused);
    }
#if defined(CONFIG_CPU_AARCH32_CORTEX_R)
    pos += snprintk(line + pos, sizeof(line) - pos, " svc=%u irq=%u",
                    (unsigned)mon_unused((const uint8_t *)K_KERNEL_STACK_BUFFER(z_arm_svc_stack[0]),
                                         CONFIG_ARMV7_SVC_STACK_SIZE),
                    (unsigned)mon_unused((const uint8_t *)K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]),
                                         CONFIG_ISR_STACK_SIZE));
#endif
    printk("%s\n", line);
}
#endif // MON_HAVE_STACKS

// the table built by main, in chunks: a long single %s may be cut by the
// formatter, and the sleeps keep the burst from holding main off for long
void mon_print_table(void)
{
    for (size_t off = 0; off < mon_table_len; off += MON_CHUNK) {
        printk("%.*s", (int)MIN(MON_CHUNK, mon_table_len - off), mon_table + off);
        k_sleep(K_MSEC(2));
    }
    printk("\n");
}

void mon_fn(void *, void *, void *)
{
    uint32_t periods = 0;
    uint32_t lines = 0;

    for (;;) {
        k_sleep(K_MSEC(CONFIG_AP_HAL_ZEPHYR_MONITOR_PERIOD_MS));

        if (atomic_get(&mon_table_ready)) {
            mon_print_table();
            atomic_set(&mon_table_ready, 0);
        }
        if (++periods % CONFIG_AP_HAL_ZEPHYR_MONITOR_PRINT_EVERY != 0) {
            continue;
        }
        lines++;

        // read-only from here on. This thread preempts main and does not
        // block between the reads: main cannot reset perf_info in between.
        const AP_Scheduler *sched = AP_Scheduler::get_singleton();
        const bool init = hal.scheduler->is_system_initialized();
        uint32_t ticks = 0, loops = 0, max_us = 0, avg_us = 0, nlong = 0;
        if (sched != nullptr) {
            ticks = sched->ticks32();
            loops = sched->perf_info.get_num_loops();
            max_us = sched->perf_info.get_max_time();
            // get_avg_time() divides by the loop count: 0 right after a
            // reset, or when the main loop does not run
            avg_us = loops != 0 ? sched->perf_info.get_avg_time() : 0;
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
#if CONFIG_AP_HAL_ZEPHYR_DELAY_SLEEP_MIN_US > 0
        printk(" dl=%u/%u", (unsigned)ap_hal_zephyr_delay_late_count,
               (unsigned)ap_hal_zephyr_delay_late_max_us);
#endif
#if HAL_ZEPHYR_RPMSG_SERIAL_ENABLED
        struct ap_rpmsg_serial_stats rp;
        ap_rpmsg_serial_stats_get(&rp);
        printk(" rp rx=%u tx=%u st=%u dr=%u no=%u ki=%u", (unsigned)rp.rx_bytes,
               (unsigned)rp.tx_bytes, (unsigned)rp.tx_stalls, (unsigned)rp.rx_dropped,
               (unsigned)rp.notifies, (unsigned)rp.kicks);
#endif
        printk("\n");
        mon_cpu_line();
#if MON_HAVE_STACKS
        if (lines % CONFIG_AP_HAL_ZEPHYR_MONITOR_STACKS_EVERY == 0) {
            mon_stack_line();
        }
#endif
        // requests to main; a request main has not served yet is left alone
        if (init && sched != nullptr && CONFIG_AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY > 0 &&
            atomic_get(&mon_request) == REQ_NONE && !atomic_get(&mon_table_ready)) {
            if (!mon_tasks_enabled) {
                atomic_set(&mon_request, REQ_ENABLE);
            } else if (lines % CONFIG_AP_HAL_ZEPHYR_MONITOR_TASKS_EVERY == 0) {
                atomic_set(&mon_request, REQ_TABLE);
            }
        }
    }
}

K_THREAD_STACK_DEFINE(mon_stack, 4096);
struct k_thread mon_thread_data;

} // namespace

/*
 * MAIN thread, once per main loop (HAL_Zephyr::run()): serve the monitor's
 * request. Costs one atomic read per loop when there is none; a served
 * REQ_TABLE formats the task table (floats, heap for the ExpandingString):
 * expect one long main loop per table.
 */
void ap_hal_zephyr_monitor_main_service(void)
{
    const int req = (int)atomic_get(&mon_request);
    if (req == REQ_NONE) {
        return;
    }
    AP_Scheduler *sched = AP_Scheduler::get_singleton();
    if (sched != nullptr) {
        if (req == REQ_ENABLE) {
            ExpandingString tmp;
            sched->task_info(tmp); // the first call sets RECORD_TASK_INFO
            if (!sched->perf_info.has_task_info()) {
                sched->perf_info.allocate_task_info(255);
            }
            sched->perf_info.reset();
            mon_tasks_enabled = true;
        } else if (req == REQ_TABLE) {
            ExpandingString str;
            sched->task_info(str);
            size_t len = 0;
            if (!str.has_failed_allocation() && str.get_length() != 0) {
                len = MIN((size_t)str.get_length(), sizeof(mon_table));
                memcpy(mon_table, str.get_string(), len);
            }
            // reset after the snapshot: the next table and the loop figures
            // cover the next interval (update_logging() does this when
            // logging is built)
            sched->perf_info.reset();
            mon_table_len = len;
            if (len != 0) {
                atomic_set(&mon_table_ready, 1);
            }
        }
    }
    atomic_set(&mon_request, REQ_NONE);
}

// called from HAL_Zephyr::run() (a SYS_INIT here would be dropped: nothing
// else references this object in the static HAL library)
void ap_hal_zephyr_monitor_start(void)
{
    k_thread_create(&mon_thread_data, mon_stack, K_THREAD_STACK_SIZEOF(mon_stack), mon_fn,
                    nullptr, nullptr, nullptr, K_PRIO_PREEMPT(MON_PRIO), 0, K_NO_WAIT);
    k_thread_name_set(&mon_thread_data, "ap_mon");
}

#endif
