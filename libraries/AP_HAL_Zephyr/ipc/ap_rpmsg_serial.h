/*
 * Byte-stream bridge between the OpenAMP rpmsg endpoint (rpmsg_remote.c) and
 * the AP_HAL_Zephyr RPMsgUARTDriver. The driver (C++, compiled by waf without
 * the OpenAMP include path) talks to the rpmsg layer (C, compiled into the
 * Zephyr image) only through this byte-buffer API -- no OpenAMP/libmetal types
 * cross the boundary.
 *
 * Threading contract:
 *  - ap_rpmsg_serial_write() may be called from any thread (MAVLink in the
 *    main thread, hal.console from others): it is NON-BLOCKING, serialises
 *    the writers with irq_lock() and returns how many bytes were accepted
 *    into the TX ring (the rpmsg mng thread drains that ring into the rpmsg
 *    buffer pool with rpmsg_trysend).
 *  - ap_rpmsg_serial_read()/_rxavail()/_discard_rx() are for ONE reading
 *    thread; the RX ring is filled by the rpmsg rx callback in the mng thread.
 * Producer against consumer needs no lock (Zephyr ring_buf).
 *
 * SPDX-License-Identifier: (GPL-2.0 OR MIT)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool ap_rpmsg_serial_up(void);

uint32_t ap_rpmsg_serial_read(uint8_t *dst, uint32_t len);

uint32_t ap_rpmsg_serial_rxavail(void);

uint32_t ap_rpmsg_serial_write(const uint8_t *src, uint32_t len);

uint32_t ap_rpmsg_serial_txspace(void);

bool ap_rpmsg_serial_tx_pending(void);

void ap_rpmsg_serial_discard_rx(void);

/* link counters for diagnostics (bytes, kicks, stalled sends, dropped RX) */
struct ap_rpmsg_serial_stats {
	uint32_t rx_bytes, tx_bytes, rx_dropped, tx_stalls, notifies, kicks;
};
void ap_rpmsg_serial_stats_get(struct ap_rpmsg_serial_stats *st);

#ifdef __cplusplus
}
#endif
