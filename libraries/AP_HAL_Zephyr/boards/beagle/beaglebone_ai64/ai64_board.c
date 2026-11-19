/*
 * BeagleBone AI-64 main_r5f1_0 board glue for AP_HAL_Zephyr.
 *
 * remoteproc graceful shutdown: Linux k3_r5_remoteproc (TI 6.12 kernel, split
 * mode) sends RP_MBOX_SHUTDOWN on the mailbox before it stops the core. Mask
 * the mailbox user, quiesce the VIM, ACK with RP_MBOX_SHUTDOWN_ACK and WFI
 * forever (TI PDK ipc_testsetup.c's sequence); the core is reset after that.
 *
 * Before any of that the motor outputs go to their safe state: the eHRPWM
 * modules and GPIO2 are not reset with the core and would keep the last
 * duty / enable / brake-release levels.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util_macro.h>

#include "../../../SafeState.h"

#define RP_MBOX_SHUTDOWN     0xFFFFFF14u
#define RP_MBOX_SHUTDOWN_ACK 0xFFFFFF15u

#define MBOX_BASE  DT_REG_ADDR(DT_NODELABEL(mbox2))
#define MBOX_USER  DT_PROP(DT_NODELABEL(mbox2), usr_id)
#define MBOX_IRQ_ENABLE_CLEAR(u) (MBOX_BASE + 0x100u + 0x10u * (u) + 0xcu)
#define VIM_REG(off) (DT_REG_ADDR(DT_INST(0, ti_vim)) + (off))
#define VIM_GROUPS   (CONFIG_NUM_IRQS / 32u)

/* tries and pause for the shutdown ACK while the mailbox FIFO is full */
#define SHUTDOWN_ACK_TRIES    200
#define SHUTDOWN_ACK_PAUSE_US 50u

/*
 * The board's safe state (strong definition of the HAL hook, see
 * SafeState.h): raw register writes, ISR-safe, repeatable.
 *  - every eHRPWM output forced low at once: AQSFRC (0x1a) RLDCSF = 3
 *    (AQCSFRC loads immediately), AQCSFRC (0x1c) = 0x5 (A low, B low,
 *    continuous software force);
 *  - every GPIO2 pin an input (DIR, offset 0 of each davinci bank, 1 =
 *    input): the board's pull-downs then disarm the output stage, disable
 *    the bridges and apply the brakes.
 * Only for the nodes the devicetree enables: with the current overlay (no
 * motor layer yet, ehrpwm0/4/5 and main_gpio2_* disabled) this compiles to
 * nothing. The forced-low and input states persist until a driver
 * re-initialises the peripherals: the state is latched, as it must be.
 */
void ap_hal_zephyr_outputs_safe(void)
{
#define AI64_PWM_OFF(label)                                                      \
	IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(label)),                 \
		   (sys_write16(0x00c0u, DT_REG_ADDR(DT_NODELABEL(label)) + 0x1au); \
		    sys_write16(0x0005u, DT_REG_ADDR(DT_NODELABEL(label)) + 0x1cu);))
	AI64_PWM_OFF(ehrpwm0)
	AI64_PWM_OFF(ehrpwm4)
	AI64_PWM_OFF(ehrpwm5)
#define AI64_GPIO_IN(label)                                                      \
	IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(label)),                 \
		   (sys_write32(0xffffffffu, DT_REG_ADDR(DT_NODELABEL(label)));))
	AI64_GPIO_IN(main_gpio2_0)
	AI64_GPIO_IN(main_gpio2_1)
	AI64_GPIO_IN(main_gpio2_2)
	AI64_GPIO_IN(main_gpio2_3)
}

void ap_rpmsg_board_ipm_msg(const struct device *dev, uint32_t id, volatile void *data)
{
	uint32_t ack = RP_MBOX_SHUTDOWN_ACK;
	int ret = -EBUSY;

	ARG_UNUSED(id);
	if (data == NULL || *(volatile uint32_t *)data != RP_MBOX_SHUTDOWN) {
		return;
	}
	(void)irq_lock();
	/* first: Linux stops (resets) this core once it has the ACK */
	ap_hal_zephyr_outputs_safe();
	sys_write32(0xffffffffu, MBOX_IRQ_ENABLE_CLEAR(MBOX_USER));
	for (uint32_t g = 0; g < VIM_GROUPS; g++) {
		sys_write32(0xffffffffu, VIM_REG(0x40cu + 0x20u * g)); /* INTR_EN_CLR */
		sys_write32(0xffffffffu, VIM_REG(0x404u + 0x20u * g)); /* STS */
		sys_write32(0xffffffffu, VIM_REG(0x410u + 0x20u * g)); /* IRQSTS */
	}
	printk("ai64: RP_MBOX_SHUTDOWN: outputs safe, ACK + WFI\n");
	/*
	 * The TX FIFO may be full of vring kicks Linux has not read yet
	 * (-EBUSY): retry for up to 10 ms, then give up. Without the ACK Linux
	 * times out and stops the core anyway; the outputs are already safe.
	 */
	for (int i = 0; i < SHUTDOWN_ACK_TRIES; i++) {
		ret = ipm_send(dev, 0, 0, &ack, sizeof(ack));
		if (ret != -EBUSY) {
			break;
		}
		k_busy_wait(SHUTDOWN_ACK_PAUSE_US);
	}
	if (ret != 0) {
		printk("ai64: shutdown ACK not sent (%d)\n", ret);
	}
	for (;;) {
		__asm__ volatile("dsb\n\twfi" ::: "memory");
	}
}
