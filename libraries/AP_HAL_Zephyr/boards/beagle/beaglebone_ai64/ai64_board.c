/*
 * BeagleBone AI-64 main_r5f1_0 board glue for AP_HAL_Zephyr.
 *
 * remoteproc graceful shutdown: Linux k3_r5_remoteproc (TI 6.12 kernel, split
 * mode) sends RP_MBOX_SHUTDOWN on the mailbox before it stops the core. Mask
 * the mailbox user, quiesce the VIM, ACK with RP_MBOX_SHUTDOWN_ACK and WFI
 * forever (TI PDK ipc_testsetup.c's sequence); the core is reset after that.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/ipm.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/sys_io.h>

#define RP_MBOX_SHUTDOWN     0xFFFFFF14u
#define RP_MBOX_SHUTDOWN_ACK 0xFFFFFF15u

#define MBOX_BASE  DT_REG_ADDR(DT_NODELABEL(mbox2))
#define MBOX_USER  DT_PROP(DT_NODELABEL(mbox2), usr_id)
#define MBOX_IRQ_ENABLE_CLEAR(u) (MBOX_BASE + 0x100u + 0x10u * (u) + 0xcu)
#define VIM_REG(off) (DT_REG_ADDR(DT_INST(0, ti_vim)) + (off))
#define VIM_GROUPS   (CONFIG_NUM_IRQS / 32u)

void ap_rpmsg_board_ipm_msg(const struct device *dev, uint32_t id, volatile void *data)
{
	uint32_t ack = RP_MBOX_SHUTDOWN_ACK;

	ARG_UNUSED(id);
	if (data == NULL || *(volatile uint32_t *)data != RP_MBOX_SHUTDOWN) {
		return;
	}
	(void)irq_lock();
	sys_write32(0xffffffffu, MBOX_IRQ_ENABLE_CLEAR(MBOX_USER));
	for (uint32_t g = 0; g < VIM_GROUPS; g++) {
		sys_write32(0xffffffffu, VIM_REG(0x40cu + 0x20u * g)); /* INTR_EN_CLR */
		sys_write32(0xffffffffu, VIM_REG(0x404u + 0x20u * g)); /* STS */
		sys_write32(0xffffffffu, VIM_REG(0x410u + 0x20u * g)); /* IRQSTS */
	}
	printk("ai64: RP_MBOX_SHUTDOWN: ACK + WFI\n");
	(void)ipm_send(dev, 0, 0, &ack, sizeof(ack));
	for (;;) {
		__asm__ volatile("dsb\n\twfi" ::: "memory");
	}
}
