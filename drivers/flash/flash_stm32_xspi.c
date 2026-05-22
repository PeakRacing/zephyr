/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * **************************************************************************
 * xSPI flash controller driver for stm32 series with xSPI periherals
 * This driver is based on the stm32Cube HAL XSPI driver
 * with one xspi DTS NODE
 * **************************************************************************
 */
#define DT_DRV_COMPAT st_stm32_xspi_nor

#include <errno.h>
#include <zephyr/kernel.h>
#include <soc.h>
#include <stm32_bitops.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/dt-bindings/flash_controller/xspi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>

#include "spi_nor.h"
#include "jesd216.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_stm32_xspi, CONFIG_FLASH_LOG_LEVEL);

#define STM32_XSPI_NODE(inst) DT_INST_PARENT(inst)

#define STM32_XSPI_RESET_GPIO DT_ANY_INST_HAS_PROP_STATUS_OKAY(reset_gpios)

#ifdef CONFIG_FLASH_STM32_XSPI_DMA
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/drivers/dma.h>
#include <stm32_ll_dma.h>
#endif /* CONFIG_FLASH_STM32_XSPI_DMA */

#if defined(CONFIG_SOC_SERIES_STM32H7RSX)
#include <stm32_ll_pwr.h>
#include <stm32_ll_system.h>
#endif /* CONFIG_SOC_SERIES_STM32H7RSX */

#include "flash_stm32_xspi.h"

/* ============================================================
 * XIP-SAFE DEBUG HELPERS
 *
 * All symbols below are relocated to AXISRAM1 together with the rest of
 * this translation unit (CMakeLists zephyr_code_relocate LOCATION RAM).
 * They intentionally access ONLY memory-mapped system registers and the
 * USART1 peripheral — never XIP flash — so they are safe to call while
 * the XSPI controller is in command mode (FMODE ≠ 0b11, XIP disabled).
 *
 * USART1 on STM32N647 — Secure world address:
 *   peripherals node: ranges = <0x0 0x50000000 0x10000000>
 *   usart1: serial@2001000 → 0x50000000 + 0x2001000 = 0x52001000
 *   ISR  offset 0x1C  bit[7] = TXFNF (TX FIFO/register not full)
 *   TDR  offset 0x28
 * ============================================================
 */
#ifdef CONFIG_STM32_MEMMAP

#define XSPI_DBG_UART_BASE  0x52001000UL
#define XSPI_DBG_UART_ISR (*(volatile uint32_t *)(XSPI_DBG_UART_BASE + 0x1CUL))
#define XSPI_DBG_UART_TDR (*(volatile uint32_t *)(XSPI_DBG_UART_BASE + 0x28UL))

static void xspi_dbg_putc(char c)
{
//	/* Spin until TX register/FIFO has room (TXFNF, bit 7).
//	 * Timeout guard of 200 000 iterations (~250 µs @ 800 MHz) prevents
//	 * an infinite loop if USART1 is not yet configured.
/	 */
//	volatile uint32_t t = 200000U;
//
//	while (!(XSPI_DBG_UART_ISR & (1U << 7U)) && --t) {
//	}
//	XSPI_DBG_UART_TDR = (uint8_t)c;
}

static void xspi_dbg_puts(const char *s)
{
//	while (*s) {
//		xspi_dbg_putc(*s++);
//	}
}

static void xspi_dbg_hex32(uint32_t v)
{
//	xspi_dbg_putc('0');
//	xspi_dbg_putc('x');
//	for (int i = 28; i >= 0; i -= 4) {
//		uint8_t n = (v >> i) & 0xFU;
//
//		xspi_dbg_putc(n < 10U ? '0' + n : 'A' + n - 10U);
//	}
}

/* ── RAM vector table ──────────────────────────────────────────────────────
 * Populated just before XIP is disabled so that any exception during the
 * XIP-off window uses AXISRAM1 handlers rather than XIP (disabled) → LOCKUP.
 *
 * Cortex-M55: 16 core + ≤240 external = 256 entries max, 1024-byte table.
 * VTOR must be aligned to next power-of-2 ≥ table size → 1024-byte aligned.
 */
#define XSPI_VTABLE_ENTRIES 256U
static uint32_t xspi_vtable_ram[XSPI_VTABLE_ENTRIES] __attribute__((aligned(1024U)));
static uint32_t xspi_orig_vtor;

/* Pointer to the XSPI instance CR register; set before entering the
 * critical section so the HardFault handler can read it without arguments.
 */
static volatile uint32_t *xspi_dbg_cr_ptr;

/* ── Debug HardFault handler (AXISRAM1) ────────────────────────────────────
 * Two-layer design so we can capture the faulting PC/LR from the exception
 * frame even though the function is called as an ordinary C function.
 *
 * xspi_debug_hardfault_c: C body that receives the frame pointer in r0.
 *   frame layout (Cortex-M, no FP stacking):
 *     [0]=r0 [1]=r1 [2]=r2 [3]=r3 [4]=r12 [5]=lr [6]=pc [7]=xpsr
 *
 * xspi_debug_hardfault_entry: naked trampoline stored in the vector table.
 *   Detects whether the interrupted context used MSP or PSP (EXC_RETURN bit 2),
 *   passes the correct frame pointer in r0, then branches to the C handler.
 *
 * XSPI2 CR — Secure address:
 *   AHB5PERIPH_BASE_S = PERIPH_BASE_S(0x50000000) + 0x08020000 = 0x58020000
 *   XSPI2_BASE_S = AHB5PERIPH_BASE_S + 0xA000 = 0x5802A000
 */
__attribute__((used)) static void xspi_debug_hardfault_c(uint32_t *frame)
{
	uint32_t pc   = frame ? frame[6] : 0xDEADBEEFU;
	uint32_t lr   = frame ? frame[5] : 0xDEADBEEFU;
	uint32_t hfsr = SCB->HFSR;
	uint32_t cfsr = SCB->CFSR;
	uint32_t bfar = SCB->BFAR;
	uint32_t xspi_cr = xspi_dbg_cr_ptr ? *xspi_dbg_cr_ptr : 0xDEADBEEFU;

	/* Clear W1C sticky bits */
	SCB->HFSR = hfsr;
	SCB->CFSR = cfsr;

	/* Wait for any in-progress UART DMA TX to settle (~10 ms guard) */
	volatile uint32_t dly = 8000000U;

	while (--dly) {
	}

	xspi_dbg_puts("\r\n*** XSPI HardFault ***\r\n");
	xspi_dbg_puts("PC=");    xspi_dbg_hex32(pc);
	xspi_dbg_puts(" LR=");   xspi_dbg_hex32(lr);
	xspi_dbg_puts(" HFSR="); xspi_dbg_hex32(hfsr);
	xspi_dbg_puts(" CFSR="); xspi_dbg_hex32(cfsr);
	xspi_dbg_puts(" BFAR="); xspi_dbg_hex32(bfar);
	xspi_dbg_puts(" XCR=");  xspi_dbg_hex32(xspi_cr);
	xspi_dbg_puts(" VTOR="); xspi_dbg_hex32(SCB->VTOR);
	xspi_dbg_puts("\r\n");

	/* Halt — do NOT reset, keep UART output visible */
	while (1) {
	}
}

/* Naked trampoline stored in the vector table.  EXC_RETURN bit 2 selects the
 * stack that held the interrupted context (0 = MSP, 1 = PSP).
 */
__attribute__((naked)) static void xspi_debug_hardfault_entry(void)
{
	__asm__ volatile(
		"tst  lr, #4\n"
		"ite  eq\n"
		"mrseq r0, msp\n"
		"mrsne r0, psp\n"
		"b    xspi_debug_hardfault_c\n"
	);
}

/* xspi_vtable_install: update the CR pointer used by the debug handler.
 * The VTOR was already redirected to xspi_vtable_ram at boot by
 * xspi_vtable_install_early() (SYS_INIT, POST_KERNEL, 0), so there is no
 * need to copy or relocate the vector table here — just refresh the pointer.
 */
static void xspi_vtable_install(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	/* Update CR pointer with the actual device instance address. */
	xspi_dbg_cr_ptr = (volatile uint32_t *)&dev_data->hxspi.Instance->CR;
}

/* xspi_vtable_install_early: called at POST_KERNEL,0 (before main()).
 * Copies the current XIP vector table to xspi_vtable_ram in AXISRAM1,
 * replaces NMI/HardFault/MemManage with the AXISRAM1 handler, and points
 * SCB->VTOR at the RAM copy.  From this moment any exception that fires —
 * including during the XIP-disabled window in erase/write — uses our
 * AXISRAM1 handler and will NOT cause an XIP fetch → LOCKUP double-fault.
 *
 * XSPI2 CR Secure address (hardcoded fallback before flash driver init):
 *   PERIPH_BASE_S = 0x50000000, AHB5PERIPH_BASE_S += 0x08020000 = 0x58020000
 *   XSPI2_BASE_S = AHB5PERIPH_BASE_S + 0xA000 = 0x5802A000
 */
static int xspi_vtable_install_early(void)
{
	/* Hardcoded XSPI2 CR (Secure) as fallback before flash_stm32_xspi_init */
	xspi_dbg_cr_ptr = (volatile uint32_t *)0x5802A000UL;

	/* Capture original VTOR before touching it */
	xspi_orig_vtor = SCB->VTOR;
	const uint32_t *src = (const uint32_t *)xspi_orig_vtor;

	for (uint32_t i = 0U; i < XSPI_VTABLE_ENTRIES; i++) {
		xspi_vtable_ram[i] = src[i];
	}

	/* Override non-maskable entries with the AXISRAM1 naked trampoline:
	 *   [2] NMI       (priority -2, never maskable by PRIMASK)
	 *   [3] HardFault (priority -1, never maskable by PRIMASK)
	 *   [4] MemManage (may fire before escalating to HardFault)
	 */
	xspi_vtable_ram[2] = (uint32_t)xspi_debug_hardfault_entry; /* NMI */
	xspi_vtable_ram[3] = (uint32_t)xspi_debug_hardfault_entry; /* HardFault */
	xspi_vtable_ram[4] = (uint32_t)xspi_debug_hardfault_entry; /* MemManage */
	xspi_vtable_ram[5] = (uint32_t)xspi_debug_hardfault_entry; /* BusFault */
	xspi_vtable_ram[6] = (uint32_t)xspi_debug_hardfault_entry; /* UsageFault */
	xspi_vtable_ram[7] = (uint32_t)xspi_debug_hardfault_entry; /* SecureFault */

	__DSB();
	SCB->VTOR = (uint32_t)xspi_vtable_ram;
	__DSB();
	__ISB();

	printk("[xspi] vtable_early: VTOR=0x%08x -> RAM=0x%08x fn=0x%08x\n",
	       xspi_orig_vtor, (uint32_t)xspi_vtable_ram,
	       (uint32_t)xspi_debug_hardfault_entry);

	/* Test raw UART access RIGHT NOW (UART is up, XIP active — safe).
	 * If "[raw-uart] ok" appears, 0x52001000 is accessible.
	 * If it does NOT appear but the line above did, the Secure address is
	 * wrong — we're likely running NonSecure and need 0x42001000 instead.
	 */
	xspi_dbg_puts("[raw-uart] ok (from vtable_early)\r\n");

	return 0;
}

/* APPLICATION, 0 — serial console is guaranteed to be initialized by then.
 * LittleFS is mounted in main() which runs after ALL SYS_INIT hooks, so
 * VTOR will be in RAM before the first flash erase/write is attempted.
 */
SYS_INIT(xspi_vtable_install_early, APPLICATION, 0);

#endif /* CONFIG_STM32_MEMMAP */

#ifndef CONFIG_STM32_MEMMAP
/* No-op stubs so xspi_dbg_* calls compile when MEMMAP is disabled. */
static inline void xspi_dbg_puts(const char *s) { ARG_UNUSED(s); }
static inline void xspi_dbg_hex32(uint32_t v) { ARG_UNUSED(v); }
#endif /* !CONFIG_STM32_MEMMAP */

static inline void xspi_lock_thread(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	k_sem_take(&dev_data->sem, K_FOREVER);
}

static inline void xspi_unlock_thread(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	k_sem_give(&dev_data->sem);
}

static int xspi_send_cmd(const struct device *dev, XSPI_RegularCmdTypeDef *cmd)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;
	HAL_StatusTypeDef hal_ret;

	dev_data->cmd_status = 0;

	hal_ret = HAL_XSPI_Command(&dev_data->hxspi, cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		return -EIO;  /* No LOG: called while XIP may be disabled */
	}

	return dev_data->cmd_status;
}

static int xspi_read_access(const struct device *dev, XSPI_RegularCmdTypeDef *cmd,
			    uint8_t *data, const size_t size)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;
	HAL_StatusTypeDef hal_ret;

	LOG_DBG("Instruction 0x%x", cmd->Instruction);

	cmd->DataLength = size;

	dev_data->cmd_status = 0;

	hal_ret = HAL_XSPI_Command(&dev_data->hxspi, cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send XSPI instruction", hal_ret);
		return -EIO;
	}

#ifdef CONFIG_FLASH_STM32_XSPI_DMA
	hal_ret = HAL_XSPI_Receive_DMA(&dev_data->hxspi, data);
#else
	hal_ret = HAL_XSPI_Receive_IT(&dev_data->hxspi, data);
#endif

	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

static int xspi_write_access(const struct device *dev, XSPI_RegularCmdTypeDef *cmd,
			     const uint8_t *data, const size_t size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	HAL_StatusTypeDef hal_ret;

	cmd->DataLength = size;

	dev_data->cmd_status = 0;

#if defined(CONFIG_STM32_MEMMAP)
   /* in OPI/STR the 3-byte AddressWidth is not supported by the NOR flash */
   xspi_dbg_puts("[wa] dm="); xspi_dbg_hex32(dev_cfg->data_mode);
   xspi_dbg_puts(" aw="); xspi_dbg_hex32(cmd->AddressWidth);
   xspi_dbg_puts(" dl="); xspi_dbg_hex32((uint32_t)size); xspi_dbg_puts("\r\n");
   if ((dev_cfg->data_mode == XSPI_OCTO_MODE) &&
           (cmd->AddressWidth != HAL_XSPI_ADDRESS_32_BITS)) {
           xspi_dbg_puts("[wa] early-exit: OctoMode+!32bit\r\n");
           return -EIO;
   }
#endif /* CONFIG_STM32_MEMMAP */

   hal_ret = HAL_XSPI_Command(&dev_data->hxspi, cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
#if defined(CONFIG_STM32_MEMMAP)
   xspi_dbg_puts("[wa] cmd ret="); xspi_dbg_hex32((uint32_t)hal_ret);
   xspi_dbg_puts(" ec="); xspi_dbg_hex32(dev_data->hxspi.ErrorCode);
   xspi_dbg_puts(" st="); xspi_dbg_hex32((uint32_t)dev_data->hxspi.State); xspi_dbg_puts("\r\n");
#endif /* CONFIG_STM32_MEMMAP */
   if (hal_ret != HAL_OK) {
           return -EIO;
   }

#if defined(CONFIG_STM32_MEMMAP)
   hal_ret = HAL_XSPI_Transmit(&dev_data->hxspi, (uint8_t *)data, HAL_MAX_DELAY);
   xspi_dbg_puts("[wa] tx ret="); xspi_dbg_hex32((uint32_t)hal_ret);
   xspi_dbg_puts(" ec="); xspi_dbg_hex32(dev_data->hxspi.ErrorCode);
   xspi_dbg_puts(" st="); xspi_dbg_hex32((uint32_t)dev_data->hxspi.State); xspi_dbg_puts("\r\n");
   if (hal_ret != HAL_OK) {
           return -EIO;
   }
   return dev_data->cmd_status;
#endif /* CONFIG_STM32_MEMMAP */

#ifdef CONFIG_FLASH_STM32_XSPI_DMA
	hal_ret = HAL_XSPI_Transmit_DMA(&dev_data->hxspi, (uint8_t *)data);
#else
	hal_ret = HAL_XSPI_Transmit_IT(&dev_data->hxspi, (uint8_t *)data);
#endif

	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to write data", hal_ret);
		return -EIO;
	}

	k_sem_take(&dev_data->sync, K_FOREVER);

	return dev_data->cmd_status;
}

/*
 * Zero a memory region without calling memset().
 * In XIP configurations picolibc memset lives in XIP flash; calls from
 * RAM-relocated code go through __memset_veneer -> XIP address.  While
 * FMODE=0 (XIP disabled for erase/write) that branch faults (IBUSERR).
 * Use this helper to zero structs inside the XIP-disabled critical section.
 */
static void xspi_zero_mem(void *p, size_t n)
{
	uint8_t *b = (uint8_t *)p;

	while (n--) {
		*b++ = 0;
	}
}

/*
 * Gives a XSPI_RegularCmdTypeDef with all parameters set
 * except Instruction, Address, DummyCycles, NbData
 */
static void xspi_prepare_cmd(XSPI_RegularCmdTypeDef *cmd, const uint8_t transfer_mode,
					       const uint8_t transfer_rate)
{
	/* Fill *cmd directly: returning by value emits __memcpy_veneer->XIP (fatal while XIP disabled) */


	xspi_zero_mem(cmd, sizeof(*cmd));
	cmd->OperationType    = HAL_XSPI_OPTYPE_COMMON_CFG;
	cmd->InstructionWidth = ((transfer_mode == XSPI_OCTO_MODE)
				? HAL_XSPI_INSTRUCTION_16_BITS
				: HAL_XSPI_INSTRUCTION_8_BITS);
	cmd->InstructionDTRMode = ((transfer_rate == XSPI_DTR_TRANSFER)
				? HAL_XSPI_INSTRUCTION_DTR_ENABLE
				: HAL_XSPI_INSTRUCTION_DTR_DISABLE);
	cmd->AddressDTRMode   = ((transfer_rate == XSPI_DTR_TRANSFER)
				? HAL_XSPI_ADDRESS_DTR_ENABLE
				: HAL_XSPI_ADDRESS_DTR_DISABLE);
	/* AddressWidth must be set to 32bits for init and mem config phase */
	cmd->AddressWidth     = HAL_XSPI_ADDRESS_32_BITS;
	cmd->AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	cmd->DataDTRMode      = ((transfer_rate == XSPI_DTR_TRANSFER)
				? HAL_XSPI_DATA_DTR_ENABLE
				: HAL_XSPI_DATA_DTR_DISABLE);
	cmd->DQSMode          = (transfer_rate == XSPI_DTR_TRANSFER)
				? HAL_XSPI_DQS_ENABLE
				: HAL_XSPI_DQS_DISABLE;
#ifdef XSPI_CCR_SIOO
	cmd->SIOOMode         = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif /* XSPI_CCR_SIOO */

	switch (transfer_mode) {
	case XSPI_OCTO_MODE: {
		cmd->InstructionMode = HAL_XSPI_INSTRUCTION_8_LINES;
		cmd->AddressMode = HAL_XSPI_ADDRESS_8_LINES;
		cmd->DataMode = HAL_XSPI_DATA_8_LINES;
		break;
	}
	case XSPI_QUAD_MODE: {
		cmd->InstructionMode = HAL_XSPI_INSTRUCTION_4_LINES;
		cmd->AddressMode = HAL_XSPI_ADDRESS_4_LINES;
		cmd->DataMode = HAL_XSPI_DATA_4_LINES;
		break;
	}
	case XSPI_DUAL_MODE: {
		cmd->InstructionMode = HAL_XSPI_INSTRUCTION_2_LINES;
		cmd->AddressMode = HAL_XSPI_ADDRESS_2_LINES;
		cmd->DataMode = HAL_XSPI_DATA_2_LINES;
		break;
	}
	default: {
		cmd->InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		cmd->AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		cmd->DataMode = HAL_XSPI_DATA_1_LINE;
		break;
	}
	}


}

static uint32_t stm32_xspi_hal_address_size(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	if (dev_data->address_width == 4U) {
		return HAL_XSPI_ADDRESS_32_BITS;
	}

	return HAL_XSPI_ADDRESS_24_BITS;
}

#if defined(CONFIG_FLASH_JESD216_API)
/*
 * Read the JEDEC ID data from the external Flash at init
 * and store in the jedec_id Table of the flash_stm32_xspi_data
 * The JEDEC ID is not given by a DTS property
 */
static int stm32_xspi_read_jedec_id(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	/* This is a SPI/STR command to issue to the external Flash device */
	XSPI_RegularCmdTypeDef cmd;
	xspi_prepare_cmd(&cmd, XSPI_SPI_MODE, XSPI_STR_TRANSFER);

	cmd.Instruction = JESD216_CMD_READ_ID;
	cmd.AddressWidth = stm32_xspi_hal_address_size(dev);
	cmd.AddressMode = HAL_XSPI_ADDRESS_NONE;
	cmd.DataLength = JESD216_READ_ID_LEN; /* 3 bytes in the READ ID */

	HAL_StatusTypeDef hal_ret;

	hal_ret = HAL_XSPI_Command(&dev_data->hxspi, &cmd,
				   HAL_XSPI_TIMEOUT_DEFAULT_VALUE);

	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send XSPI instruction", hal_ret);
		return -EIO;
	}

	/* Place the received data directly into the jedec Table */
	hal_ret = HAL_XSPI_Receive(&dev_data->hxspi, dev_data->jedec_id,
				   HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

	LOG_DBG("Jedec ID = [%02x %02x %02x]",
		dev_data->jedec_id[0], dev_data->jedec_id[1], dev_data->jedec_id[2]);

	dev_data->cmd_status = 0;

	return 0;
}

/*
 * Read Serial Flash ID :
 * just gives the values received by the external Flash
 */
static int xspi_read_jedec_id(const struct device *dev,  uint8_t *id)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	/* Take jedec Id values from the table (issued from the octoFlash) */
	memcpy(id, dev_data->jedec_id, JESD216_READ_ID_LEN);

	LOG_INF("Manuf ID = %02x   Memory Type = %02x   Memory Density = %02x",
		id[0], id[1], id[2]);

	return 0;
}
#endif /* CONFIG_FLASH_JESD216_API */

/*
 * Read Serial Flash Discovery Parameter from the external Flash at init :
 * perform a read access over SPI bus for SDFP (DataMode is already set)
 * The SFDP table is not given by a DTS property
 */
static int stm32_xspi_read_sfdp(const struct device *dev, off_t addr,
				void *data,
				size_t size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;

	XSPI_RegularCmdTypeDef cmd;
	xspi_prepare_cmd(&cmd, dev_cfg->data_mode, dev_cfg->data_rate);
	if (dev_cfg->data_mode == XSPI_OCTO_MODE) {
		cmd.Instruction = JESD216_OCMD_READ_SFDP;
		cmd.DummyCycles = 20U;
		cmd.AddressWidth = HAL_XSPI_ADDRESS_32_BITS;
	} else {
		cmd.Instruction = JESD216_CMD_READ_SFDP;
		cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		cmd.DataMode = HAL_XSPI_DATA_1_LINE;
		cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		cmd.DummyCycles = 8U;
		cmd.AddressWidth = HAL_XSPI_ADDRESS_24_BITS;
	}
	cmd.Address = addr;
	cmd.DataLength = size;

	HAL_StatusTypeDef hal_ret;

	hal_ret = HAL_XSPI_Command(&dev_data->hxspi, &cmd, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to send XSPI instruction", hal_ret);
		return -EIO;
	}

	hal_ret = HAL_XSPI_Receive(&dev_data->hxspi, (uint8_t *)data,
				   HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("%d: Failed to read data", hal_ret);
		return -EIO;
	}

	dev_data->cmd_status = 0;

	return 0;
}

/*
 * Read Serial Flash Discovery Parameter :
 * perform a read access over SPI bus for SDFP (DataMode is already set)
 */
static int xspi_read_sfdp(const struct device *dev, off_t addr, void *data,
			  size_t size)
{
	LOG_INF("Read SFDP from externalFlash");
	/* Get the SFDP from the external Flash (no sfdp-bfp table in the DeviceTree) */
	if (stm32_xspi_read_sfdp(dev, addr, data, size) == 0) {
		/* If valid, then ignore any table from the DTS */
		return 0;
	}
	LOG_INF("Error reading SFDP from external Flash and none in the DTS");
	return -EINVAL;
}

static bool xspi_address_is_valid(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	size_t flash_size = dev_cfg->flash_size;

	return (addr >= 0) && ((uint64_t)addr + (uint64_t)size <= flash_size);
}

static int stm32_xspi_wait_auto_polling(const struct device *dev,
		XSPI_AutoPollingTypeDef *s_config, uint32_t timeout_ms)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

#if defined(CONFIG_STM32_MEMMAP) || \
	(defined(CONFIG_STM32_APP_IN_EXT_FLASH) && defined(CONFIG_XIP))
	/* XIP-SAFE PATCH: Use blocking HAL_XSPI_AutoPolling() instead of the
	 * interrupt-based variant + k_sem_take().  Both HAL_XSPI_AutoPolling_IT
	 * and k_sem_take() call Zephyr kernel functions that live in XIP flash.
	 * When the XSPI controller is in command/auto-polling mode (FMODE cleared)
	 * any instruction fetch from XIP causes a hard fault → cold reset.
	 * HAL_XSPI_AutoPolling() is entirely in RAM (code-relocated) and
	 * XSPI_WaitFlagStateUntilTimeout() has been patched to use a loop counter
	 * instead of HAL_GetTick(), so the whole call chain is XIP-free. */
	ARG_UNUSED(timeout_ms);
	if (HAL_XSPI_AutoPolling(&dev_data->hxspi, s_config,
				  HAL_MAX_DELAY) != HAL_OK) {
		/* No LOG: called while XIP may be disabled */
		return -EIO;
	}
	return 0;
#else
	dev_data->cmd_status = 0;

	if (HAL_XSPI_AutoPolling_IT(&dev_data->hxspi, s_config) != HAL_OK) {
		LOG_ERR("XSPI AutoPoll failed");
		return -EIO;
	}

	if (k_sem_take(&dev_data->sync, K_MSEC(timeout_ms)) != 0) {
		LOG_ERR("XSPI AutoPoll wait failed");
		if (HAL_XSPI_Abort(&dev_data->hxspi) != HAL_OK) {
			LOG_ERR("XSPI abort failed");
		}
		k_sem_reset(&dev_data->sync);
		return -EIO;
	}

	/* HAL_XSPI_AutoPolling_IT enables transfer error interrupt which sets
	 * cmd_status.
	 */
	return dev_data->cmd_status;
#endif /* CONFIG_STM32_MEMMAP || (CONFIG_STM32_APP_IN_EXT_FLASH && CONFIG_XIP) */
}

/*
 * This function Polls the WEL (write enable latch) bit to become to 0
 * When the Chip Erase Cycle is completed, the Write Enable Latch (WEL) bit is cleared.
 * in nor_mode SPI/OPI XSPI_SPI_MODE or XSPI_OCTO_MODE
 * and nor_rate transfer STR/DTR XSPI_STR_TRANSFER or XSPI_DTR_TRANSFER
 */
static int stm32_xspi_mem_erased(const struct device *dev,
		uint8_t nor_mode, uint8_t nor_rate)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	XSPI_AutoPollingTypeDef s_config;
	xspi_zero_mem(&s_config, sizeof(s_config));
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);

	/* Configure automatic polling mode command to wait for memory ready */
	if (nor_mode == XSPI_OCTO_MODE) {
		s_command.Instruction = SPI_NOR_OCMD_RDSR;
		s_command.DummyCycles = (nor_rate == XSPI_DTR_TRANSFER)
					? SPI_NOR_DUMMY_REG_OCTAL_DTR
					: SPI_NOR_DUMMY_REG_OCTAL;
	} else {
		s_command.Instruction = SPI_NOR_CMD_RDSR;
		/* force 1-line InstructionMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		s_command.AddressMode = HAL_XSPI_ADDRESS_NONE;
		/* force 1-line DataMode for any non-OSPI transfer */
		s_command.DataMode = HAL_XSPI_DATA_1_LINE;
		s_command.DummyCycles = 0;
	}
	s_command.DataLength = ((nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U);
	s_command.Address = 0U;

	/* Set the mask to  0x02 to mask all Status REG bits except WEL */
	/* Set the match to 0x00 to check if the WEL bit is Reset */
	s_config.MatchValue         = SPI_NOR_WEL_MATCH;
	s_config.MatchMask          = SPI_NOR_WEL_MASK; /* Write Enable Latch */

	s_config.MatchMode          = HAL_XSPI_MATCH_MODE_AND;
	s_config.IntervalTime       = SPI_NOR_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop      = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_XSPI_Command(&dev_data->hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		/* No LOG: called while XIP may be disabled */
		return -EIO;
	}

	/* Start Automatic-Polling mode to wait until the memory is totally erased */
	return stm32_xspi_wait_auto_polling(dev,
			&s_config, STM32_XSPI_BULK_ERASE_MAX_TIME);
}

/*
 * This function Polls the WIP(Write In Progress) bit to become to 0
 * in nor_mode SPI/OPI XSPI_SPI_MODE or XSPI_OCTO_MODE
 * and nor_rate transfer STR/DTR XSPI_STR_TRANSFER or XSPI_DTR_TRANSFER
 */
static int stm32_xspi_mem_ready(const struct device *dev, uint8_t nor_mode,
		uint8_t nor_rate)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	XSPI_AutoPollingTypeDef s_config;
	xspi_zero_mem(&s_config, sizeof(s_config));
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);

	/* Configure automatic polling mode command to wait for memory ready */
	if (nor_mode == XSPI_OCTO_MODE) {
		s_command.Instruction = SPI_NOR_OCMD_RDSR;
		s_command.DummyCycles = (nor_rate == XSPI_DTR_TRANSFER)
					? SPI_NOR_DUMMY_REG_OCTAL_DTR
					: SPI_NOR_DUMMY_REG_OCTAL;
	} else {
		s_command.Instruction = SPI_NOR_CMD_RDSR;
		/* force 1-line InstructionMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		s_command.AddressMode = HAL_XSPI_ADDRESS_NONE;
		/* force 1-line DataMode for any non-OSPI transfer */
		s_command.DataMode = HAL_XSPI_DATA_1_LINE;
		s_command.DummyCycles = 0;
	}
	s_command.DataLength = ((nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U);
	s_command.Address = 0U;

	/* Set the mask to  0x01 to mask all Status REG bits except WIP */
	/* Set the match to 0x00 to check if the WIP bit is Reset */
	s_config.MatchValue         = SPI_NOR_MEM_RDY_MATCH;
	s_config.MatchMask          = SPI_NOR_MEM_RDY_MASK; /* Write in progress */
	s_config.MatchMode          = HAL_XSPI_MATCH_MODE_AND;
	s_config.IntervalTime       = SPI_NOR_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop      = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_XSPI_Command(&dev_data->hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		/* No LOG: called while XIP may be disabled */
		return -EIO;
	}

	/* Start Automatic-Polling mode to wait until the memory is ready WIP=0 */
	return stm32_xspi_wait_auto_polling(dev, &s_config, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

/* Enables writing to the memory sending a Write Enable and wait it is effective */
static int stm32_xspi_write_enable(const struct device *dev,
		uint8_t nor_mode, uint8_t nor_rate)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	XSPI_AutoPollingTypeDef s_config;
	xspi_zero_mem(&s_config, sizeof(s_config));
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);

	/* Initialize the write enable command */
	if (nor_mode == XSPI_OCTO_MODE) {
		s_command.Instruction = SPI_NOR_OCMD_WREN;
	} else {
		s_command.Instruction = SPI_NOR_CMD_WREN;
		/* force 1-line InstructionMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	}
	s_command.AddressMode = HAL_XSPI_ADDRESS_NONE;
	s_command.DataMode    = HAL_XSPI_DATA_NONE;
	s_command.DummyCycles = 0U;

	if (HAL_XSPI_Command(&dev_data->hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		/* No LOG: called while XIP may be disabled */
		return -EIO;
	}

	/* New command to Configure automatic polling mode to wait for write enabling */
	if (nor_mode == XSPI_OCTO_MODE) {
		s_command.Instruction = SPI_NOR_OCMD_RDSR;
		s_command.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
		s_command.DataMode = HAL_XSPI_DATA_8_LINES;
		s_command.DummyCycles = (nor_rate == XSPI_DTR_TRANSFER)
				? SPI_NOR_DUMMY_REG_OCTAL_DTR
						: SPI_NOR_DUMMY_REG_OCTAL;
	} else {
		s_command.Instruction = SPI_NOR_CMD_RDSR;
		/* force 1-line DataMode for any non-OSPI transfer */
		s_command.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
		s_command.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
		s_command.DataMode = HAL_XSPI_DATA_1_LINE;
		s_command.DummyCycles = 0;

		/* DummyCycles remains 0 */
	}
	s_command.DataLength = (nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U;
	s_command.Address = 0U;

	if (HAL_XSPI_Command(&dev_data->hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		/* No LOG: called while XIP may be disabled */
		return -EIO;
	}

	s_config.MatchValue      = SPI_NOR_WREN_MATCH;
	s_config.MatchMask       = SPI_NOR_WREN_MASK;
	s_config.MatchMode       = HAL_XSPI_MATCH_MODE_AND;
	s_config.IntervalTime    = SPI_NOR_AUTO_POLLING_INTERVAL;
	s_config.AutomaticStop   = HAL_XSPI_AUTOMATIC_STOP_ENABLE;

	return stm32_xspi_wait_auto_polling(dev, &s_config, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
}

static int xspi_write_unprotect(const struct device *dev)
{
	int ret = 0;

	/* This is a SPI/STR command to issue to the external Flash device */
	XSPI_RegularCmdTypeDef cmd_unprotect;
	xspi_prepare_cmd(&cmd_unprotect, XSPI_SPI_MODE, XSPI_STR_TRANSFER);

	cmd_unprotect.Instruction = SPI_NOR_CMD_ULBPR;
	cmd_unprotect.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
	cmd_unprotect.AddressMode = HAL_XSPI_ADDRESS_NONE;
	cmd_unprotect.DataMode    = HAL_XSPI_DATA_NONE;

	ret = stm32_xspi_write_enable(dev, XSPI_SPI_MODE, XSPI_STR_TRANSFER);
	if (ret != 0) {
		return ret;
	}

	return xspi_send_cmd(dev, &cmd_unprotect);
}

/* Write Flash configuration register 2 with new dummy cycles */
static int stm32_xspi_write_cfg2reg_dummy(const struct device *dev,
					uint8_t nor_mode, uint8_t nor_rate)
{
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	uint8_t transmit_data;

	if (dev_cfg->max_frequency == MHZ(200)) {
		/* Use memory default value */
		return 0;
	}

	transmit_data = SPI_NOR_CR2_DUMMY_CYCLES_66MHZ;

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (nor_mode == XSPI_SPI_MODE)
				? SPI_NOR_CMD_WR_CFGREG2
				: SPI_NOR_OCMD_WR_CFGREG2;
	s_command.Address = SPI_NOR_REG2_ADDR3;
	s_command.DummyCycles = 0U;
	s_command.DataLength = (nor_mode == XSPI_SPI_MODE) ? 1U
			: ((nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U);

	if (HAL_XSPI_Command(&dev_data->hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI transmit cmd");
		return -EIO;
	}

	if (HAL_XSPI_Transmit(&dev_data->hxspi, &transmit_data,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI transmit ");
		return -EIO;
	}

	return 0;
}

/* Write Flash configuration register 2 with new single or octal SPI protocol */
static int stm32_xspi_write_cfg2reg_io(XSPI_HandleTypeDef *hxspi,
				       uint8_t nor_mode, uint8_t nor_rate, uint8_t op_enable)
{
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (nor_mode == XSPI_SPI_MODE)
				? SPI_NOR_CMD_WR_CFGREG2
				: SPI_NOR_OCMD_WR_CFGREG2;
	s_command.Address = SPI_NOR_REG2_ADDR1;
	s_command.DummyCycles = 0U;
	s_command.DataLength = (nor_mode == XSPI_SPI_MODE) ? 1U
		: ((nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U);

	if (HAL_XSPI_Command(hxspi, &s_command,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	if (HAL_XSPI_Transmit(hxspi, &op_enable,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return 0;
}

/* Read Flash configuration register 2 with new single or octal SPI protocol */
static int stm32_xspi_read_cfg2reg(XSPI_HandleTypeDef *hxspi,
				   uint8_t nor_mode, uint8_t nor_rate, uint8_t *value)
{
	XSPI_RegularCmdTypeDef s_command;
	xspi_prepare_cmd(&s_command, nor_mode, nor_rate);

	/* Initialize the writing of configuration register 2 */
	s_command.Instruction = (nor_mode == XSPI_SPI_MODE)
				? SPI_NOR_CMD_RD_CFGREG2
				: SPI_NOR_OCMD_RD_CFGREG2;
	s_command.Address = SPI_NOR_REG2_ADDR1;
	s_command.DummyCycles = (nor_mode == XSPI_SPI_MODE)
				? 0U
				: ((nor_rate == XSPI_DTR_TRANSFER)
					? SPI_NOR_DUMMY_REG_OCTAL_DTR
					: SPI_NOR_DUMMY_REG_OCTAL);
	s_command.DataLength = (nor_rate == XSPI_DTR_TRANSFER) ? 2U : 1U;

	if (HAL_XSPI_Command(hxspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	if (HAL_XSPI_Receive(hxspi, value, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("Write Flash configuration reg2 failed");
		return -EIO;
	}

	return 0;
}

/* Set the NOR Flash to desired Interface mode : SPI/OSPI and STR/DTR according to the DTS */
static int stm32_xspi_config_mem(const struct device *dev)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	uint8_t reg[2];

	/* Going to set the SPI mode and STR transfer rate : done */
	if ((dev_cfg->data_mode != XSPI_OCTO_MODE)
		&& (dev_cfg->data_rate == XSPI_STR_TRANSFER)) {
		LOG_INF("OSPI flash config is SPI|DUAL|QUAD / STR");
		return 0;
	}

	/* Going to set the XPI mode (STR or DTR transfer rate) */
	LOG_DBG("XSPI configuring Octo SPI mode");

	if (stm32_xspi_write_enable(dev,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER) != 0) {
		LOG_ERR("OSPI write Enable failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with new dummy cycles) */
	if (stm32_xspi_write_cfg2reg_dummy(dev,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER) != 0) {
		LOG_ERR("XSPI write CFGR2 failed");
		return -EIO;
	}
	if (stm32_xspi_mem_ready(dev,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER) != 0) {
		LOG_ERR("XSPI autopolling failed");
		return -EIO;
	}
	if (stm32_xspi_write_enable(dev,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER) != 0) {
		LOG_ERR("XSPI write Enable 2 failed");
		return -EIO;
	}

	/* Write Configuration register 2 (with Octal I/O SPI protocol : choose STR or DTR) */
	uint8_t mode_enable = ((dev_cfg->data_rate == XSPI_DTR_TRANSFER)
				? SPI_NOR_CR2_DTR_OPI_EN
				: SPI_NOR_CR2_STR_OPI_EN);
	if (stm32_xspi_write_cfg2reg_io(&dev_data->hxspi,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER, mode_enable) != 0) {
		LOG_ERR("XSPI write CFGR2 failed");
		return -EIO;
	}

	/* Wait that the configuration is effective and check that memory is ready */
	k_busy_wait(STM32_XSPI_WRITE_REG_MAX_TIME * USEC_PER_MSEC);

	/* Reconfigure the memory type of the peripheral */
	dev_data->hxspi.Init.MemoryType            = HAL_XSPI_MEMTYPE_MACRONIX;
	dev_data->hxspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
	if (HAL_XSPI_Init(&dev_data->hxspi) != HAL_OK) {
		LOG_ERR("XSPI mem type MACRONIX failed");
		return -EIO;
	}

	if (dev_cfg->data_rate == XSPI_STR_TRANSFER) {
		if (stm32_xspi_mem_ready(dev,
			XSPI_OCTO_MODE, XSPI_STR_TRANSFER) != 0) {
			/* Check Flash busy ? */
			LOG_ERR("XSPI flash busy failed");
			return -EIO;
		}

		if (stm32_xspi_read_cfg2reg(&dev_data->hxspi,
			XSPI_OCTO_MODE, XSPI_STR_TRANSFER, reg) != 0) {
			/* Check the configuration has been correctly done on SPI_NOR_REG2_ADDR1 */
			LOG_ERR("XSPI flash config read failed");
			return -EIO;
		}

		LOG_INF("XSPI flash config is OCTO / STR");
	}

	if (dev_cfg->data_rate == XSPI_DTR_TRANSFER) {
		if (stm32_xspi_mem_ready(dev,
			XSPI_OCTO_MODE, XSPI_DTR_TRANSFER) != 0) {
			/* Check Flash busy ? */
			LOG_ERR("XSPI flash busy failed");
			return -EIO;
		}

		if (stm32_xspi_read_cfg2reg(&dev_data->hxspi,
			XSPI_OCTO_MODE, XSPI_DTR_TRANSFER, reg) != 0) {
			/* Check the configuration has been correctly done on SPI_NOR_REG2_ADDR1 */
			LOG_ERR("XSPI flash config read failed");
			return -EIO;
		}

		LOG_INF("XSPI flash config is OCTO / DTR");
	}

	return 0;
}

/* gpio or send the different reset command to the NOR flash in SPI/OSPI and STR/DTR */
static int stm32_xspi_mem_reset(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

#if STM32_XSPI_RESET_GPIO
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;

	/* Generate RESETn pulse for the flash memory */
	gpio_pin_configure_dt(&dev_cfg->reset, GPIO_OUTPUT_ACTIVE);
	k_msleep(dev_cfg->reset_gpios_duration);
	gpio_pin_set_dt(&dev_cfg->reset, 0);
#else

	/* Reset command sent successively for each mode SPI/OPS & STR/DTR */
	XSPI_RegularCmdTypeDef s_command = {
		.OperationType = HAL_XSPI_OPTYPE_COMMON_CFG,
		.AddressMode = HAL_XSPI_ADDRESS_NONE,
		.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE,
		.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE,
		.Instruction = SPI_NOR_CMD_RESET_EN,
		.InstructionWidth = HAL_XSPI_INSTRUCTION_8_BITS,
		.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE,
		.DataLength = HAL_XSPI_DATA_NONE,
		.DummyCycles = 0U,
		.DQSMode = HAL_XSPI_DQS_DISABLE,
#ifdef XSPI_CCR_SIOO
		.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD,
#endif /* XSPI_CCR_SIOO */
	};

	/* Reset enable in SPI mode and STR transfer mode */
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset enable (SPI/STR) failed");
		return -EIO;
	}

	/* Reset memory in SPI mode and STR transfer mode */
	s_command.Instruction = SPI_NOR_CMD_RESET_MEM;
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset memory (SPI/STR) failed");
		return -EIO;
	}

	/* Reset enable in OPI mode and STR transfer mode */
	s_command.InstructionMode    = HAL_XSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	s_command.Instruction = SPI_NOR_OCMD_RESET_EN;
	s_command.InstructionWidth = HAL_XSPI_INSTRUCTION_16_BITS;
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset enable (OCTO/STR) failed");
		return -EIO;
	}

	/* Reset memory in OPI mode and STR transfer mode */
	s_command.Instruction = SPI_NOR_OCMD_RESET_MEM;
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset memory (OCTO/STR) failed");
		return -EIO;
	}

	/* Reset enable in OPI mode and DTR transfer mode */
	s_command.InstructionDTRMode = HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	s_command.Instruction = SPI_NOR_OCMD_RESET_EN;
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset enable (OCTO/DTR) failed");
		return -EIO;
	}

	/* Reset memory in OPI mode and DTR transfer mode */
	s_command.Instruction = SPI_NOR_OCMD_RESET_MEM;
	if (HAL_XSPI_Command(&dev_data->hxspi,
		&s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI reset memory (OCTO/DTR) failed");
		return -EIO;
	}

#endif /* STM32_XSPI_RESET_GPIO */
	/* Wait after SWreset CMD, in case SWReset occurred during erase operation */
	k_busy_wait(STM32_XSPI_RESET_MAX_TIME * USEC_PER_MSEC);

	return 0;
}

#ifdef CONFIG_STM32_MEMMAP
/* Function to configure the octoflash in MemoryMapped mode */
static int stm32_xspi_set_memorymap(const struct device *dev)
{
	HAL_StatusTypeDef ret;
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	XSPI_RegularCmdTypeDef s_command; /* Use xspi_zero_mem: = {0} triggers __memset_veneer -> XIP fault */
	xspi_zero_mem(&s_command, sizeof(s_command));
	XSPI_MemoryMappedTypeDef s_MemMappedCfg;
	xspi_zero_mem(&s_MemMappedCfg, sizeof(s_MemMappedCfg));

	/* Configure octoflash in MemoryMapped mode */
	if ((dev_cfg->data_mode == XSPI_SPI_MODE) &&
		(stm32_xspi_hal_address_size(dev) == HAL_XSPI_ADDRESS_24_BITS)) {
		/* No LOG: may be called while XIP is disabled */
		return -EIO;
	}

	/* Initialize the read command */
	s_command.OperationType = HAL_XSPI_OPTYPE_READ_CFG;
	s_command.InstructionMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? HAL_XSPI_INSTRUCTION_1_LINE
					: HAL_XSPI_INSTRUCTION_8_LINES)
				: HAL_XSPI_INSTRUCTION_8_LINES;
	s_command.InstructionDTRMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? HAL_XSPI_INSTRUCTION_DTR_DISABLE
				: HAL_XSPI_INSTRUCTION_DTR_ENABLE;
	s_command.InstructionWidth = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? HAL_XSPI_INSTRUCTION_8_BITS
					: HAL_XSPI_INSTRUCTION_16_BITS)
				: HAL_XSPI_INSTRUCTION_16_BITS;
	s_command.Instruction = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? ((stm32_xspi_hal_address_size(dev) ==
					HAL_XSPI_ADDRESS_24_BITS)
						? SPI_NOR_CMD_READ_FAST
						: SPI_NOR_CMD_READ_FAST_4B)
					: dev_data->read_opcode)
				: SPI_NOR_OCMD_DTR_RD;
	s_command.AddressMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? HAL_XSPI_ADDRESS_1_LINE
					: HAL_XSPI_ADDRESS_8_LINES)
				: HAL_XSPI_ADDRESS_8_LINES;
	s_command.AddressDTRMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? HAL_XSPI_ADDRESS_DTR_DISABLE
				: HAL_XSPI_ADDRESS_DTR_ENABLE;
	s_command.AddressWidth = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? stm32_xspi_hal_address_size(dev)
				: HAL_XSPI_ADDRESS_32_BITS;
	s_command.DataMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? HAL_XSPI_DATA_1_LINE
					: HAL_XSPI_DATA_8_LINES)
				: HAL_XSPI_DATA_8_LINES;
	s_command.DataDTRMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? HAL_XSPI_DATA_DTR_DISABLE
				: HAL_XSPI_DATA_DTR_ENABLE;
	s_command.DummyCycles = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? ((dev_cfg->data_mode == XSPI_SPI_MODE)
					? SPI_NOR_DUMMY_RD
					: SPI_NOR_DUMMY_RD_OCTAL)
				: SPI_NOR_DUMMY_RD_OCTAL_DTR;
	s_command.DQSMode = (dev_cfg->data_rate == XSPI_STR_TRANSFER)
				? HAL_XSPI_DQS_DISABLE
				: HAL_XSPI_DQS_ENABLE;
#ifdef XSPI_CCR_SIOO
	s_command.SIOOMode = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif /* XSPI_CCR_SIOO */

	ret = HAL_XSPI_Command(&dev_data->hxspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (ret != HAL_OK) {
		/* No LOG: may be called while XIP is disabled */
		return -EIO;
	}

	/* Initialize the program command */
	s_command.OperationType = HAL_XSPI_OPTYPE_WRITE_CFG;
	if (dev_cfg->data_rate == XSPI_STR_TRANSFER) {
		s_command.Instruction = (dev_cfg->data_mode == XSPI_SPI_MODE)
					? ((stm32_xspi_hal_address_size(dev) ==
					HAL_XSPI_ADDRESS_24_BITS)
						? SPI_NOR_CMD_PP
						: SPI_NOR_CMD_PP_4B)
					: SPI_NOR_OCMD_PAGE_PRG;
	} else {
		s_command.Instruction = SPI_NOR_OCMD_PAGE_PRG;
	}
	s_command.DQSMode = HAL_XSPI_DQS_DISABLE;

	ret = HAL_XSPI_Command(&dev_data->hxspi, &s_command, HAL_XSPI_TIMEOUT_DEFAULT_VALUE);
	if (ret != HAL_OK) {
		/* No LOG: may be called while XIP is disabled */
		return -EIO;
	}

	/* Enable the memory-mapping */
	s_MemMappedCfg.TimeOutActivation = HAL_XSPI_TIMEOUT_COUNTER_DISABLE;

#ifdef XSPI_CR_NOPREF
	s_MemMappedCfg.NoPrefetchData = HAL_XSPI_AUTOMATIC_PREFETCH_ENABLE;
#ifdef XSPI_CR_NOPREF_AXI
	s_MemMappedCfg.NoPrefetchAXI = HAL_XSPI_AXI_PREFETCH_DISABLE;
#endif /* XSPI_CR_NOPREF_AXI */
#endif /* XSPI_CR_NOPREF */

	ret = HAL_XSPI_MemoryMapped(&dev_data->hxspi, &s_MemMappedCfg);
	if (ret != HAL_OK) {
		/* No LOG: may be called while XIP is disabled */
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_STM32_MEMMAP */

static int stm32_xspi_abort(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	if (HAL_XSPI_Abort(&dev_data->hxspi) != HAL_OK) {
		/* No LOG: may be called while XIP is disabled */
		return -EIO;
	}

	return 0;
}

/* Function to return true if the octoflash is in MemoryMapped else false */
static bool stm32_xspi_is_memorymap(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	return stm32_reg_read_bits(&dev_data->hxspi.Instance->CR, XSPI_CR_FMODE) == XSPI_CR_FMODE;
}

/*
 * Function to erase the flash : chip or sector with possible OCTO/SPI and STR/DTR
 * to erase the complete chip (using dedicated command) :
 *   set size >= flash size
 *   set addr = 0
 */
static int flash_stm32_xspi_erase(const struct device *dev, off_t addr,
				  size_t size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	int ret = 0;
	/* Saved PRIMASK for XIP-disabled critical section (irq_lock/irq_unlock).
	 * irq_unlock(0) is a no-op when interrupts were already enabled, so
	 * calling irq_unlock(irq_key) unconditionally at erase_end is safe.
	 */
	unsigned int irq_key = 0;

	/* Ignore zero size erase */
	if (size == 0) {
		return 0;
	}

	/* Maximise erase size : means the complete chip */
	if (size > dev_cfg->flash_size) {
		size = dev_cfg->flash_size;
	}

	if (!xspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	if (((size % SPI_NOR_SECTOR_SIZE) != 0) && (size < dev_cfg->flash_size)) {
		LOG_ERR("Error: wrong sector size 0x%x", size);
		return -ENOTSUP;
	}

	xspi_lock_thread(dev);

	/*
	 * XIP-RODATA SHADOW: copy config + shadow struct device before disabling
	 * XIP.  After stm32_xspi_abort() clears FMODE the 0x70000000 window is
	 * gone; any load from dev (XIP .rodata) or dev_cfg (XIP .rodata) faults.
	 * We create stack copies here (XIP still active) and use dev_s +
	 * cfg_shadow for every call in the critical section.
	 */
	struct flash_stm32_xspi_config cfg_shadow = *dev_cfg;
	uint32_t __aligned(4) dev_shadow_raw[(sizeof(struct device) + 3U) / 4U];
	memcpy(dev_shadow_raw, dev, sizeof(struct device));
	{
		const void *p = &cfg_shadow;
		memcpy((uint8_t *)dev_shadow_raw +
			       offsetof(struct device, config),
		       &p, sizeof(p));
	}
	const struct device *dev_s = (const struct device *)dev_shadow_raw;

#ifdef CONFIG_STM32_MEMMAP
	{
		bool is_mm = stm32_xspi_is_memorymap(dev_s);

		if (is_mm) {
			irq_key = irq_lock();
			SCB_DisableICache();

			/* VTOR already points to RAM (installed at APPLICATION,0).
			 * xspi_vtable_install() only updates xspi_dbg_cr_ptr.
			 */
			xspi_vtable_install(dev_s);
			xspi_dbg_puts("\r\n[XSP-E] vtable=");
			xspi_dbg_hex32((uint32_t)xspi_vtable_ram);
			xspi_dbg_puts(" orig_vtor=");
			xspi_dbg_hex32(xspi_orig_vtor);

			/* Abort ongoing transfer to force CS high/BUSY deasserted */
			__disable_irq(); /* PRIMASK=1: mask ALL irqs (incl. SysTick) during XIP-disabled window */
			xspi_dbg_puts(" abort...");
			ret = stm32_xspi_abort(dev_s);
			if (ret != 0) {
				xspi_dbg_puts(" ABORT_FAIL\r\n");
				goto erase_end;
			}
			xspi_dbg_puts(" ok\r\n");
		}
	}
#endif /* CONFIG_STM32_MEMMAP */

	/* Use xspi_zero_mem: '= {...}' emits __memset_veneer->XIP call (fatal if XIP disabled) */
	XSPI_RegularCmdTypeDef cmd_erase;

	xspi_zero_mem(&cmd_erase, sizeof(cmd_erase));
	cmd_erase.OperationType      = HAL_XSPI_OPTYPE_COMMON_CFG;
	cmd_erase.AlternateBytesMode = HAL_XSPI_ALT_BYTES_NONE;
	cmd_erase.DataMode           = HAL_XSPI_DATA_NONE;
	cmd_erase.DummyCycles        = 0U;
	cmd_erase.DQSMode            = HAL_XSPI_DQS_DISABLE;
#ifdef XSPI_CCR_SIOO
	cmd_erase.SIOOMode           = HAL_XSPI_SIOO_INST_EVERY_CMD;
#endif /* XSPI_CCR_SIOO */

	if (stm32_xspi_mem_ready(dev_s,
		cfg_shadow.data_mode, cfg_shadow.data_rate) != 0) {
		ret = -EIO; /* No LOG: XIP is disabled */
		goto erase_end;
	}

	cmd_erase.InstructionMode    = (cfg_shadow.data_mode == XSPI_OCTO_MODE)
					? HAL_XSPI_INSTRUCTION_8_LINES
					: HAL_XSPI_INSTRUCTION_1_LINE;
	cmd_erase.InstructionDTRMode = (cfg_shadow.data_rate == XSPI_DTR_TRANSFER)
					? HAL_XSPI_INSTRUCTION_DTR_ENABLE
					: HAL_XSPI_INSTRUCTION_DTR_DISABLE;
	cmd_erase.InstructionWidth    = (cfg_shadow.data_mode == XSPI_OCTO_MODE)
					? HAL_XSPI_INSTRUCTION_16_BITS
					: HAL_XSPI_INSTRUCTION_8_BITS;

	while ((size > 0) && (ret == 0)) {

		ret = stm32_xspi_write_enable(dev_s,
			cfg_shadow.data_mode, cfg_shadow.data_rate);
		if (ret != 0) {
			/* No LOG: XIP is disabled */
			break;
		}

		if (size == cfg_shadow.flash_size) {
			/* Chip erase */
			cmd_erase.Address = 0;
			cmd_erase.Instruction = (cfg_shadow.data_mode == XSPI_OCTO_MODE)
					? SPI_NOR_OCMD_BULKE
					: SPI_NOR_CMD_BULKE;
			cmd_erase.AddressMode = HAL_XSPI_ADDRESS_NONE;
			/* Full chip erase (Bulk) command */
			xspi_send_cmd(dev_s, &cmd_erase);

			size -= cfg_shadow.flash_size;
			/* Chip (Bulk) erase started, wait until WEL becomes 0 */
			ret = stm32_xspi_mem_erased(dev_s,
					cfg_shadow.data_mode, cfg_shadow.data_rate);
			if (ret != 0) {
				/* No LOG: XIP is disabled */
				break;
			}
		} else {
			/* Sector or Block erase depending on the size */

			cmd_erase.AddressMode =
				(cfg_shadow.data_mode == XSPI_OCTO_MODE)
				? HAL_XSPI_ADDRESS_8_LINES
				: HAL_XSPI_ADDRESS_1_LINE;
			cmd_erase.AddressDTRMode =
				(cfg_shadow.data_rate == XSPI_DTR_TRANSFER)
				? HAL_XSPI_ADDRESS_DTR_ENABLE
				: HAL_XSPI_ADDRESS_DTR_DISABLE;
			cmd_erase.AddressWidth = stm32_xspi_hal_address_size(dev_s);
			cmd_erase.Address = addr;

			const struct jesd216_erase_type *erase_types =
							dev_data->erase_types;
			const struct jesd216_erase_type *bet = NULL;

			for (uint8_t ei = 0;
				ei < JESD216_NUM_ERASE_TYPES; ++ei) {
				const struct jesd216_erase_type *etp =
							&erase_types[ei];

				if ((etp->exp != 0)
				    && SPI_NOR_IS_ALIGNED(addr, etp->exp)
				    && (size >= BIT(etp->exp))
				    && ((bet == NULL)
					|| (etp->exp > bet->exp))) {
					bet = etp;
					cmd_erase.Instruction = bet->cmd;
				} else if (bet == NULL) {
					/* Use the default sector erase cmd */
					if (cfg_shadow.data_mode == XSPI_OCTO_MODE) {
						cmd_erase.Instruction = SPI_NOR_OCMD_SE;
					} else {
						cmd_erase.Instruction =
							(stm32_xspi_hal_address_size(dev_s) ==
							HAL_XSPI_ADDRESS_32_BITS)
							? SPI_NOR_CMD_SE_4B
							: SPI_NOR_CMD_SE;
					}
				}
				/* Avoid using wrong erase type,
				 * if zero entries are found in erase_types
				 */
				bet = NULL;
			}
			/* No LOG: XIP is disabled in this critical section */

			xspi_send_cmd(dev_s, &cmd_erase);

			if (bet != NULL) {
				addr += BIT(bet->exp);
				size -= BIT(bet->exp);
			} else {
				addr += SPI_NOR_SECTOR_SIZE;
				size -= SPI_NOR_SECTOR_SIZE;
			}

			ret = stm32_xspi_mem_ready(dev_s, cfg_shadow.data_mode,
						cfg_shadow.data_rate);
		}

	}
	/* Ends the erase operation */

erase_end:
#ifdef CONFIG_STM32_MEMMAP
	/* Re-enable memory-mapped mode so XIP-executing callers can resume. */
	if (!stm32_xspi_is_memorymap(dev_s)) {
		xspi_dbg_puts("[XSP-E] restoring memmap...");
		if (stm32_xspi_set_memorymap(dev_s) != 0) {
			/* Fallback: force FMODE bits directly; HAL cmd-regs still
			 * valid from MCUBoot setup.  This keeps XIP alive even if
			 * the full HAL re-configure failed.
			 */
			SET_BIT(dev_data->hxspi.Instance->CR, XSPI_CR_FMODE);
			dev_data->hxspi.State = HAL_XSPI_STATE_BUSY_MEM_MAPPED;
			xspi_dbg_puts(" fallback");
		}
		xspi_dbg_puts(" CR=");
		xspi_dbg_hex32((uint32_t)dev_data->hxspi.Instance->CR);
		xspi_dbg_puts("\r\n");
	}
	/* VTOR stays in RAM (installed at boot by SYS_INIT) — do NOT restore. */
	/* Invalidate D-cache: after indirect-mode erase, XIP reads would return stale
	 * cached data (pre-erase content). LittleFS reads back after format/commit,
	 * so stale cache causes CRC mismatch and 'block unwritable' errors. */
	SCB_InvalidateDCache();
	/* Re-enable I-cache now that XIP is live again. */
	SCB_EnableICache();
	__enable_irq(); /* PRIMASK=0: safe, XIP re-enabled by set_memorymap above */
	/* XIP is back up — safe to take interrupts again. */
	irq_unlock(irq_key);
	printk("[xspi] erase done: CR=0x%08x ret=%d\n",
		(uint32_t)dev_data->hxspi.Instance->CR, ret);
#endif /* CONFIG_STM32_MEMMAP */
	xspi_unlock_thread(dev);

	return ret;
}

/* Function to read the flash with possible OCTO/SPI and STR/DTR */
static int flash_stm32_xspi_read(const struct device *dev, off_t addr,
				 void *data, size_t size)
{
	__maybe_unused const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	__maybe_unused struct flash_stm32_xspi_data *dev_data = dev->data;
	int ret = 0;

	if (!xspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	/* Ignore zero size read */
	if (size == 0) {
		return 0;
	}

#if defined(CONFIG_STM32_MEMMAP) || (defined(CONFIG_STM32_APP_IN_EXT_FLASH) && defined(CONFIG_XIP))
	/*
	 * When the call is made by an app executing in external flash,
	 * skip the memory-mapped mode check
	 */
#ifdef CONFIG_STM32_MEMMAP

	/* Do reads through memory-mapping instead of indirect */
	if (!stm32_xspi_is_memorymap(dev)) {
		xspi_lock_thread(dev);
		ret = stm32_xspi_set_memorymap(dev);
		xspi_unlock_thread(dev);

		if (ret != 0) {
			LOG_ERR("READ: failed to set memory mapped");
			return ret;
		}
	}

	__ASSERT_NO_MSG(stm32_xspi_is_memorymap(dev));
#endif /* CONFIG_STM32_MEMMAP */
	uintptr_t mmap_addr = dev_cfg->mem_map_based_address + addr;

	LOG_DBG("Memory-mapped read from 0x%08lx, len %zu", mmap_addr, size);
	memcpy(data, (void *)mmap_addr, size);
	return ret;

#else /* CONFIG_STM32_MEMMAP || (CONFIG_STM32_APP_IN_EXT_FLASH && CONFIG_XIP) */

	XSPI_RegularCmdTypeDef cmd;
xspi_prepare_cmd(&cmd, dev_cfg->data_mode, dev_cfg->data_rate);

	if (dev_cfg->data_mode != XSPI_OCTO_MODE) {
		switch (dev_data->read_mode) {
		case JESD216_MODE_112: {
			cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
			cmd.DataMode = HAL_XSPI_DATA_2_LINES;
			break;
		}
		case JESD216_MODE_122: {
			cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd.AddressMode = HAL_XSPI_ADDRESS_2_LINES;
			cmd.DataMode = HAL_XSPI_DATA_2_LINES;
			break;
		}
		case JESD216_MODE_114: {
			cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
			cmd.DataMode = HAL_XSPI_DATA_4_LINES;
			break;
		}
		case JESD216_MODE_144: {
			cmd.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
			cmd.DataMode = HAL_XSPI_DATA_4_LINES;
			break;
		}
		default:
			/* use the mode from ospi_prepare_cmd */
			break;
		}
	}

	/* Instruction and DummyCycles are set below */
	cmd.Address = addr; /* AddressSize is 32bits in OPSI mode */
	cmd.AddressWidth = stm32_xspi_hal_address_size(dev);
	/* DataSize is set by the read cmd */

	/* Configure other parameters */
	if (dev_cfg->data_rate == XSPI_DTR_TRANSFER) {
		/* DTR transfer rate (==> Octal mode) */
		cmd.Instruction = SPI_NOR_OCMD_DTR_RD;
		cmd.DummyCycles = SPI_NOR_DUMMY_RD_OCTAL_DTR;
	} else {
		/* STR transfer rate */
		if (dev_cfg->data_mode == XSPI_OCTO_MODE) {
			/* OPI and STR */
			cmd.Instruction = SPI_NOR_OCMD_RD;
			cmd.DummyCycles = SPI_NOR_DUMMY_RD_OCTAL;
		} else {
			/* use SFDP:BFP read instruction */
			cmd.Instruction = dev_data->read_opcode;
			cmd.DummyCycles = dev_data->read_dummy;
			/* in SPI and STR : expecting SPI_NOR_CMD_READ_FAST_4B */
		}
	}

	LOG_DBG("XSPI: read %zu data at 0x%lx",
		size,
		(long)(dev_cfg->mem_map_based_address + addr));
	xspi_lock_thread(dev);

	ret = xspi_read_access(dev, &cmd, data, size);
	xspi_unlock_thread(dev);

	return ret;
#endif /* CONFIG_STM32_MEMMAP || (CONFIG_STM32_APP_IN_EXT_FLASH && CONFIG_XIP) */
}

/* Function to write the flash (page program) : with possible OCTO/SPI and STR/DTR */
static int flash_stm32_xspi_write(const struct device *dev, off_t addr,
				  const void *data, size_t size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	size_t to_write;
	int ret = 0;
	/* Saved PRIMASK for XIP-disabled critical section. */
	unsigned int irq_key = 0;
	/* Save first-call addr and src for post-write readback verification. */
	off_t write_addr_orig = addr;
	const uint8_t *write_data_orig = (const uint8_t *)data;

	if (!xspi_address_is_valid(dev, addr, size)) {
		LOG_ERR("Error: address or size exceeds expected values: "
			"addr 0x%lx, size %zu", (long)addr, size);
		return -EINVAL;
	}

	/* Ignore zero size write */
	if (size == 0) {
		return 0;
	}

	xspi_lock_thread(dev);

	/*
	 * XIP-RODATA SHADOW: same as flash_stm32_xspi_erase — copy config and
	 * shadow struct device to stack before aborting XIP, so all calls after
	 * stm32_xspi_abort() access only stack/RAM.
	 */
	struct flash_stm32_xspi_config cfg_shadow = *dev_cfg;
	uint32_t __aligned(4) dev_shadow_raw[(sizeof(struct device) + 3U) / 4U];
	memcpy(dev_shadow_raw, dev, sizeof(struct device));
	{
		const void *p = &cfg_shadow;
		memcpy((uint8_t *)dev_shadow_raw +
			       offsetof(struct device, config),
		       &p, sizeof(p));
	}
	const struct device *dev_s = (const struct device *)dev_shadow_raw;

#ifdef CONFIG_STM32_MEMMAP
	if (stm32_xspi_is_memorymap(dev_s)) {
		/* Disable ALL interrupts before disabling XIP (same reason as
		 * in flash_stm32_xspi_erase — ISR code lives in XIP).
		 */
		irq_key = irq_lock();
		SCB_DisableICache();

		/* Install RAM vector table with AXISRAM1 HardFault handler BEFORE
		 * disabling XIP — same reasoning as in flash_stm32_xspi_erase().
		 */
		xspi_vtable_install(dev_s);
		xspi_dbg_puts("\r\n[XSP-W] vtable=");
		xspi_dbg_hex32((uint32_t)xspi_vtable_ram);

		/* Abort ongoing transfer to force CS high/BUSY deasserted */
		__disable_irq(); /* PRIMASK=1: mask ALL irqs during XIP-disabled window */
		xspi_dbg_puts(" abort...");
		ret = stm32_xspi_abort(dev_s);
		if (ret != 0) {
			xspi_dbg_puts(" ABORT_FAIL\r\n");
			goto write_end;
		}
		xspi_dbg_puts(" ok\r\n");
	}
#endif
	/* page program for STR or DTR mode */
	XSPI_RegularCmdTypeDef cmd_pp;
	xspi_prepare_cmd(&cmd_pp, cfg_shadow.data_mode, cfg_shadow.data_rate);

	/* using 32bits address also in SPI/STR mode */
	cmd_pp.Instruction = dev_data->write_opcode;

	if (cfg_shadow.data_mode != XSPI_OCTO_MODE) {
		switch (cmd_pp.Instruction) {
		case SPI_NOR_CMD_PP_4B:
			__fallthrough;
		case SPI_NOR_CMD_PP: {
			cmd_pp.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd_pp.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
			cmd_pp.DataMode = HAL_XSPI_DATA_1_LINE;
			break;
		}
		case SPI_NOR_CMD_PP_1_1_4_4B:
			__fallthrough;
		case SPI_NOR_CMD_PP_1_1_4: {
			cmd_pp.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd_pp.AddressMode = HAL_XSPI_ADDRESS_1_LINE;
			cmd_pp.DataMode = HAL_XSPI_DATA_4_LINES;
			break;
		}
		case SPI_NOR_CMD_PP_1_4_4_4B:
			__fallthrough;
		case SPI_NOR_CMD_PP_1_4_4: {
#if defined(CONFIG_USE_MICROCHIP_QSPI_FLASH_WITH_STM32)
			/* Microchip QSPI flash uses PP_1_1_4 opcode for the PP_1_4_4 operation */
			cmd_pp.Instruction = SPI_NOR_CMD_PP_1_1_4;
#endif /* CONFIG_USE_MICROCHIP_QSPI_FLASH_WITH_STM32 */
			cmd_pp.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE;
			cmd_pp.AddressMode = HAL_XSPI_ADDRESS_4_LINES;
			cmd_pp.DataMode = HAL_XSPI_DATA_4_LINES;
			break;
		}
		default:
			/* use the mode from ospi_prepare_cmd */
			break;
		}
	}

	cmd_pp.Address = addr;
	/* OctoSPI always uses 32-bit addressing (any flash > 16MB in OctoSPI mode
	 * requires 4-byte addresses; xspi_prepare_cmd() already sets 32-bit but
	 * would be overwritten below with the SFDP-derived value which may be 0/3
	 * if SFDP init failed while the flash was already in OctoSPI XIP mode). */
	if (cfg_shadow.data_mode != XSPI_OCTO_MODE) {
		cmd_pp.AddressWidth = stm32_xspi_hal_address_size(dev_s);
	}
	/* else: keep HAL_XSPI_ADDRESS_32_BITS set by xspi_prepare_cmd() */
	cmd_pp.DummyCycles = 0U;
	/* PP writes must NOT use DQS: DQS is a read data-strobe driven by the
	 * flash device.  The memory-mapped write config (OPTYPE_WRITE_CFG) already
	 * sets DQS_DISABLE explicitly.  Using DQS_ENABLE here in indirect mode
	 * causes the controller to either drive DQS as an output or wait for it
	 * as an input; either way the flash silently ignores the write data.
	 * Root cause confirmed by XIP readback returning 0xFFFFFFFF despite wa_ok. */
	cmd_pp.DQSMode = HAL_XSPI_DQS_DISABLE;

	/* No LOG: XIP is disabled from here until write_end */

	ret = stm32_xspi_mem_ready(dev_s,
				   cfg_shadow.data_mode, cfg_shadow.data_rate);
	if (ret != 0) {
		xspi_dbg_puts("[W] mr0_fail ret="); xspi_dbg_hex32((uint32_t)ret); xspi_dbg_puts("\r\n");
		goto write_end;
	}
	xspi_dbg_puts("[W] mr0_ok\r\n");
	while ((size > 0) && (ret == 0)) {
		to_write = size;
		ret = stm32_xspi_write_enable(dev_s,
					      cfg_shadow.data_mode, cfg_shadow.data_rate);
		if (ret != 0) {
			xspi_dbg_puts("[W] we_fail ret="); xspi_dbg_hex32((uint32_t)ret); xspi_dbg_puts("\r\n");
			break;
		}
		xspi_dbg_puts("[W] we_ok\r\n");
		if (to_write >= SPI_NOR_PAGE_SIZE) {
			to_write = SPI_NOR_PAGE_SIZE;
		}

		/* Don't write across a page boundary */
		if (((addr + to_write - 1U) / SPI_NOR_PAGE_SIZE)
		    != (addr / SPI_NOR_PAGE_SIZE)) {
			to_write = SPI_NOR_PAGE_SIZE -
						(addr % SPI_NOR_PAGE_SIZE);
		}
		cmd_pp.Address = addr;

		ret = xspi_write_access(dev_s, &cmd_pp, data, to_write);
		if (ret != 0) {
			xspi_dbg_puts("[W] wa_fail ret="); xspi_dbg_hex32((uint32_t)ret); xspi_dbg_puts("\r\n");
			break;
		}
		xspi_dbg_puts("[W] wa_ok\r\n");
		size -= to_write;
		data = (const uint8_t *)data + to_write;
		addr += to_write;

		/* Configure automatic polling mode to wait for end of program */
		ret = stm32_xspi_mem_ready(dev_s,
					   cfg_shadow.data_mode, cfg_shadow.data_rate);
		if (ret != 0) {
			xspi_dbg_puts("[W] mr1_fail ret="); xspi_dbg_hex32((uint32_t)ret); xspi_dbg_puts("\r\n");
			break;
		}
		xspi_dbg_puts("[W] mr1_ok\r\n");
		/* DIAGNOSTIC: read SR after PP to confirm WEL was cleared.
		 * WEL=0 (sr0 bit1=0): PP was accepted by flash → data should be written.
		 * WEL=1 (sr0 bit1=1): PP was IGNORED by flash → command format still wrong. */
		{
			XSPI_RegularCmdTypeDef cmd_sr2;
			xspi_prepare_cmd(&cmd_sr2, cfg_shadow.data_mode, cfg_shadow.data_rate);
			cmd_sr2.Instruction = SPI_NOR_OCMD_RDSR;
			cmd_sr2.AddressMode = HAL_XSPI_ADDRESS_8_LINES;
			cmd_sr2.Address     = 0U;
			cmd_sr2.DataMode    = HAL_XSPI_DATA_8_LINES;
			cmd_sr2.DataLength  = (cfg_shadow.data_rate == XSPI_DTR_TRANSFER) ? 2U : 1U;
			cmd_sr2.DummyCycles = (cfg_shadow.data_rate == XSPI_DTR_TRANSFER)
					      ? SPI_NOR_DUMMY_REG_OCTAL_DTR : SPI_NOR_DUMMY_REG_OCTAL;
			uint8_t srbuf2[2] = {0xAA, 0xAA};
			if (HAL_XSPI_Command(&dev_data->hxspi, &cmd_sr2, HAL_MAX_DELAY) == HAL_OK &&
			    HAL_XSPI_Receive(&dev_data->hxspi, srbuf2, HAL_MAX_DELAY) == HAL_OK) {
				xspi_dbg_puts("[SR2] sr0="); xspi_dbg_hex32(srbuf2[0]);
				xspi_dbg_puts(" sr1="); xspi_dbg_hex32(srbuf2[1]); xspi_dbg_puts("\r\n");
			} else {
				xspi_dbg_puts("[SR2] fail\r\n");
			}
		}
	}
	/* Ends the write operation */

write_end:
#ifdef CONFIG_STM32_MEMMAP
	/* Re-enable memory-mapped mode so XIP-executing callers can resume. */
	if (!stm32_xspi_is_memorymap(dev_s)) {
		xspi_dbg_puts("[XSP-W] restoring memmap...");
		if (stm32_xspi_set_memorymap(dev_s) != 0) {
			/* Fallback: force FMODE bits directly; HAL cmd-regs still
			 * valid from MCUBoot setup.
			 */
			SET_BIT(dev_data->hxspi.Instance->CR, XSPI_CR_FMODE);
			dev_data->hxspi.State = HAL_XSPI_STATE_BUSY_MEM_MAPPED;
			xspi_dbg_puts(" fallback");
		}
		xspi_dbg_puts(" CR=");
		xspi_dbg_hex32((uint32_t)dev_data->hxspi.Instance->CR);
		xspi_dbg_puts("\r\n");
	}
	/* VTOR stays in RAM (installed at boot by SYS_INIT) — do NOT restore. */
	/* Invalidate D-cache: after indirect-mode write, the D-cache still holds
	 * the pre-write content for the XIP address range. LittleFS reads back the
	 * written data for CRC verification via XIP; stale cache causes mismatches
	 * that make LittleFS mark block 0 as unwritable (LFS_ERR_NOSPC). */
	SCB_InvalidateDCache();
	/* Re-enable I-cache now that XIP is live again. */
	SCB_EnableICache();
	__enable_irq(); /* PRIMASK=0: safe, XIP back up */
	/* XIP is back up — safe to take interrupts again. */
	irq_unlock(irq_key);
	printk("[xspi] write done: CR=0x%08x ret=%d\n",
		(uint32_t)dev_data->hxspi.Instance->CR, ret);
	/* DEBUG: read back first 8 bytes via XIP to verify data reached flash.
	 * Expected: first bytes of LittleFS superblock (magic 0x6C667332 "lfs2").
	 * If 0xFFFFFFFF: write didn't program.  If matches src: write OK, look elsewhere. */
	if (ret == 0 && dev_cfg->mem_map_based_address != 0U) {
		volatile const uint32_t *xip =
			(volatile const uint32_t *)(dev_cfg->mem_map_based_address +
						    (uint32_t)write_addr_orig);
		uint32_t rb0 = xip[0];
		uint32_t rb1 = xip[1];
		uint32_t src0 = 0, src1 = 0;
		if (write_data_orig) {
			__builtin_memcpy(&src0, write_data_orig + 0, 4);
			__builtin_memcpy(&src1, write_data_orig + 4, 4);
		}
		printk("[xspi] readback @0x%08x: xip=0x%08x,0x%08x src=0x%08x,0x%08x %s\n",
			(uint32_t)(dev_cfg->mem_map_based_address +
				   (uint32_t)write_addr_orig),
			rb0, rb1, src0, src1,
			(rb0 == src0 && rb1 == src1) ? "MATCH" : "MISMATCH");
	}
#endif /* CONFIG_STM32_MEMMAP */
	xspi_unlock_thread(dev);

	return ret;
}

static const struct flash_parameters flash_stm32_xspi_parameters = {
	.write_block_size = 1,
	.erase_value = 0xff
};

static const struct flash_parameters *
flash_stm32_xspi_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_stm32_xspi_parameters;
}

static void flash_stm32_xspi_isr(const struct device *dev)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	HAL_XSPI_IRQHandler(&dev_data->hxspi);
}

#if !defined(CONFIG_SOC_SERIES_STM32H7X)
/* weak function required for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}

/* weak function required for HAL compilation */
__weak HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma)
{
	return HAL_OK;
}
#endif /* !CONFIG_SOC_SERIES_STM32H7X */

/* This function is executed in the interrupt context */
#ifdef CONFIG_FLASH_STM32_XSPI_DMA
static void xspi_dma_callback(const struct device *dev, void *arg,
			 uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	ARG_UNUSED(dev);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d.", channel);
	}

	HAL_DMA_IRQHandler(hdma);
}
#endif


/*
 * Transfer Error callback.
 */
void HAL_XSPI_ErrorCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Error cb");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

/*
 * Command completed callback.
 */
void HAL_XSPI_CmdCpltCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Cmd Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Rx Transfer completed callback.
 */
void HAL_XSPI_RxCpltCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Rx Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Tx Transfer completed callback.
 */
void HAL_XSPI_TxCpltCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Tx Cplt cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Status Match callback.
 */
void HAL_XSPI_StatusMatchCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Status Match cb");

	k_sem_give(&dev_data->sync);
}

/*
 * Timeout callback.
 */
void HAL_XSPI_TimeOutCallback(XSPI_HandleTypeDef *hxspi)
{
	struct flash_stm32_xspi_data *dev_data =
		CONTAINER_OF(hxspi, struct flash_stm32_xspi_data, hxspi);

	LOG_DBG("Timeout cb");

	dev_data->cmd_status = -EIO;

	k_sem_give(&dev_data->sync);
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flash_stm32_xspi_pages_layout(const struct device *dev,
				const struct flash_pages_layout **layout,
				size_t *layout_size)
{
	struct flash_stm32_xspi_data *dev_data = dev->data;

	*layout = &dev_data->layout;
	*layout_size = 1;
}
#endif

static int flash_stm32_xspi_get_size(const struct device *dev, uint64_t *size)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;

	*size = (uint64_t)dev_cfg->flash_size;

	return 0;
}

static DEVICE_API(flash, flash_stm32_xspi_driver_api) = {
	.read = flash_stm32_xspi_read,
	.write = flash_stm32_xspi_write,
	.erase = flash_stm32_xspi_erase,
	.get_parameters = flash_stm32_xspi_get_parameters,
	.get_size = flash_stm32_xspi_get_size,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_stm32_xspi_pages_layout,
#endif
#if defined(CONFIG_FLASH_JESD216_API)
	.sfdp_read = xspi_read_sfdp,
	.read_jedec_id = xspi_read_jedec_id,
#endif /* CONFIG_FLASH_JESD216_API */
};

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static int setup_pages_layout(const struct device *dev)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *data = dev->data;
	const size_t flash_size = dev_cfg->flash_size;
	uint32_t layout_page_size = data->page_size;
	uint8_t value = 0;
	int rv = 0;

	/* Find the smallest erase size. */
	for (size_t i = 0; i < ARRAY_SIZE(data->erase_types); ++i) {
		const struct jesd216_erase_type *etp = &data->erase_types[i];

		if ((etp->cmd != 0)
		    && ((value == 0) || (etp->exp < value))) {
			value = etp->exp;
		}
	}

	uint32_t erase_size = BIT(value);

	if (erase_size == 0) {
		erase_size = SPI_NOR_SECTOR_SIZE;
	}

	/* We need layout page size to be compatible with erase size */
	if ((layout_page_size % erase_size) != 0) {
		LOG_DBG("layout page %u not compatible with erase size %u",
			layout_page_size, erase_size);
		LOG_DBG("erase size will be used as layout page size");
		layout_page_size = erase_size;
	}

	/* Warn but accept layout page sizes that leave inaccessible
	 * space.
	 */
	if ((flash_size % layout_page_size) != 0) {
		LOG_DBG("layout page %u wastes space with device size %zu",
			layout_page_size, flash_size);
	}

	data->layout.pages_size = layout_page_size;
	data->layout.pages_count = flash_size / layout_page_size;
	LOG_DBG("layout %u x %u By pages", data->layout.pages_count,
					   data->layout.pages_size);

	return rv;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static int stm32_xspi_read_status_register(const struct device *dev, uint8_t reg_num, uint8_t *reg)
{
	XSPI_RegularCmdTypeDef s_command = {
		.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE,
		.DataMode = HAL_XSPI_DATA_1_LINE,
	};

	switch (reg_num) {
	case 1U:
		s_command.Instruction = SPI_NOR_CMD_RDSR;
		break;
	case 2U:
		s_command.Instruction = SPI_NOR_CMD_RDSR2;
		break;
	case 3U:
		s_command.Instruction = SPI_NOR_CMD_RDSR3;
		break;
	default:
		return -EINVAL;
	}

	return xspi_read_access(dev, &s_command, reg, sizeof(*reg));
}

static int stm32_xspi_write_status_register(const struct device *dev, uint8_t reg_num, uint8_t reg)
{
	struct flash_stm32_xspi_data *data = dev->data;
	XSPI_RegularCmdTypeDef s_command = {
		.Instruction = SPI_NOR_CMD_WRSR,
		.InstructionMode = HAL_XSPI_INSTRUCTION_1_LINE,
		.DataMode = HAL_XSPI_DATA_1_LINE
	};
	size_t size;
	uint8_t regs[4] = { 0 };
	uint8_t *regs_p;
	int ret;

	if (reg_num == 1U) {
		size = 1U;
		regs[0] = reg;
		regs_p = &regs[0];
		/* 1 byte write clears SR2, write SR2 as well */
		if (data->qer_type == JESD216_DW15_QER_S2B1v1) {
			ret = stm32_xspi_read_status_register(dev, 2, &regs[1]);
			if (ret < 0) {
				return ret;
			}
			size = 2U;
		}
	} else if (reg_num == 2U) {
		s_command.Instruction = SPI_NOR_CMD_WRSR2;
		size = 1U;
		regs[1] = reg;
		regs_p = &regs[1];
		/* if SR2 write needs SR1 */
		if ((data->qer_type == JESD216_DW15_QER_VAL_S2B1v1) ||
		    (data->qer_type == JESD216_DW15_QER_VAL_S2B1v4) ||
		    (data->qer_type == JESD216_DW15_QER_VAL_S2B1v5)) {
			ret = stm32_xspi_read_status_register(dev, 1, &regs[0]);
			if (ret < 0) {
				return ret;
			}
			s_command.Instruction = SPI_NOR_CMD_WRSR;
			size = 2U;
			regs_p = &regs[0];
		}
	} else if (reg_num == 3U) {
		s_command.Instruction = SPI_NOR_CMD_WRSR3;
		size = 1U;
		regs[2] = reg;
		regs_p = &regs[2];
	} else {
		return -EINVAL;
	}

	return xspi_write_access(dev, &s_command, regs_p, size);
}

static int stm32_xspi_enable_qe(const struct device *dev)
{
	struct flash_stm32_xspi_data *data = dev->data;
	uint8_t qe_reg_num;
	uint8_t qe_bit;
	uint8_t reg;
	int ret;

	switch (data->qer_type) {
	case JESD216_DW15_QER_NONE:
		/* no QE bit, device detects reads based on opcode */
		return 0;
	case JESD216_DW15_QER_S1B6:
		qe_reg_num = 1U;
		qe_bit = BIT(6U);
		break;
	case JESD216_DW15_QER_S2B7:
		qe_reg_num = 2U;
		qe_bit = BIT(7U);
		break;
	case JESD216_DW15_QER_S2B1v1:
		__fallthrough;
	case JESD216_DW15_QER_S2B1v4:
		__fallthrough;
	case JESD216_DW15_QER_S2B1v5:
		__fallthrough;
	case JESD216_DW15_QER_S2B1v6:
		qe_reg_num = 2U;
		qe_bit = BIT(1U);
		break;
	default:
		return -ENOTSUP;
	}

	ret = stm32_xspi_read_status_register(dev, qe_reg_num, &reg);
	if (ret < 0) {
		return ret;
	}

	/* exit early if QE bit is already set */
	if ((reg & qe_bit) != 0U) {
		return 0;
	}

	ret = stm32_xspi_write_enable(dev, XSPI_SPI_MODE, XSPI_STR_TRANSFER);
	if (ret < 0) {
		return ret;
	}

	reg |= qe_bit;

	ret = stm32_xspi_write_status_register(dev, qe_reg_num, reg);
	if (ret < 0) {
		return ret;
	}

	ret = stm32_xspi_mem_ready(dev, XSPI_SPI_MODE, XSPI_STR_TRANSFER);
	if (ret < 0) {
		return ret;
	}

	/* validate that QE bit is set */
	ret = stm32_xspi_read_status_register(dev, qe_reg_num, &reg);
	if (ret < 0) {
		return ret;
	}

	if ((reg & qe_bit) == 0U) {
		LOG_ERR("Status Register %u [0x%02x] not set", qe_reg_num, reg);
		ret = -EIO;
	}

	return ret;
}

static void spi_nor_process_bfp_addrbytes(const struct device *dev,
					  const uint8_t jesd216_bfp_addrbytes)
{
	struct flash_stm32_xspi_data *data = dev->data;

	if ((jesd216_bfp_addrbytes == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_4B) ||
	    (jesd216_bfp_addrbytes == JESD216_SFDP_BFP_DW1_ADDRBYTES_VAL_3B4B)) {
		data->address_width = 4U;
	} else {
		data->address_width = 3U;
	}
}

static inline uint8_t spi_nor_convert_read_to_4b(const uint8_t opcode)
{
	switch (opcode) {
	case SPI_NOR_CMD_READ:
		return SPI_NOR_CMD_READ_4B;
	case SPI_NOR_CMD_DREAD:
		return SPI_NOR_CMD_DREAD_4B;
	case SPI_NOR_CMD_2READ:
		return SPI_NOR_CMD_2READ_4B;
	case SPI_NOR_CMD_QREAD:
		return SPI_NOR_CMD_QREAD_4B;
	case SPI_NOR_CMD_4READ:
		return SPI_NOR_CMD_4READ_4B;
	default:
		/* use provided */
		return opcode;
	}
}

static inline uint8_t spi_nor_convert_write_to_4b(const uint8_t opcode)
{
	switch (opcode) {
	case SPI_NOR_CMD_PP:
		return SPI_NOR_CMD_PP_4B;
	case SPI_NOR_CMD_PP_1_1_4:
		return SPI_NOR_CMD_PP_1_1_4_4B;
	case SPI_NOR_CMD_PP_1_4_4:
		return SPI_NOR_CMD_PP_1_4_4_4B;
	default:
		/* use provided */
		return opcode;
	}
}

static int spi_nor_process_bfp(const struct device *dev,
			       const struct jesd216_param_header *php,
			       const struct jesd216_bfp *bfp)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *data = dev->data;
	/* must be kept in data mode order, ignore 1-1-1 (always supported) */
	const enum jesd216_mode_type supported_read_modes[] = { JESD216_MODE_112, JESD216_MODE_122,
								JESD216_MODE_114,
								JESD216_MODE_144 };
	size_t supported_read_modes_max_idx;
	struct jesd216_erase_type *etp = data->erase_types;
	size_t idx;
	const size_t flash_size = jesd216_bfp_density(bfp) / 8U;
	struct jesd216_instr read_instr = { 0 };
	struct jesd216_bfp_dw15 dw15;

	if (flash_size != dev_cfg->flash_size) {
		LOG_DBG("Unexpected flash size: %u", flash_size);
	}

	LOG_DBG("%s: %u MiBy flash", dev->name, (uint32_t)(flash_size >> 20));

	/* Copy over the erase types, preserving their order.  (The
	 * Sector Map Parameter table references them by index.)
	 */
	memset(data->erase_types, 0, sizeof(data->erase_types));
	for (idx = 1U; idx <= ARRAY_SIZE(data->erase_types); ++idx) {
		if (jesd216_bfp_erase(bfp, idx, etp) == 0) {
			LOG_DBG("Erase %u with %02x",
					(uint32_t)BIT(etp->exp), etp->cmd);
		}
		++etp;
	}

	spi_nor_process_bfp_addrbytes(dev, jesd216_bfp_addrbytes(bfp));
	LOG_DBG("Address width: %u Bytes", data->address_width);

	/* use PP opcode based on configured data mode if nothing is set in DTS */
	if (data->write_opcode == SPI_NOR_WRITEOC_NONE) {
		switch (dev_cfg->data_mode) {
		case XSPI_OCTO_MODE:
			data->write_opcode = SPI_NOR_OCMD_PAGE_PRG;
			break;
		case XSPI_QUAD_MODE:
			data->write_opcode = SPI_NOR_CMD_PP_1_4_4;
			break;
		case XSPI_DUAL_MODE:
			data->write_opcode = SPI_NOR_CMD_PP_1_1_2;
			break;
		default:
			data->write_opcode = SPI_NOR_CMD_PP;
			break;
		}
	}

	if (dev_cfg->data_mode != XSPI_OCTO_MODE) {
		/* determine supported read modes, begin from the slowest */
		data->read_mode = JESD216_MODE_111;
		data->read_opcode = SPI_NOR_CMD_READ;
		data->read_dummy = 0U;

		if (dev_cfg->data_mode != XSPI_SPI_MODE) {
			if (dev_cfg->data_mode == XSPI_DUAL_MODE) {
				/* the index of JESD216_MODE_114 in supported_read_modes */
				supported_read_modes_max_idx = 2U;
			} else {
				supported_read_modes_max_idx = ARRAY_SIZE(supported_read_modes);
			}

			for (idx = 0U; idx < supported_read_modes_max_idx; ++idx) {
				if (jesd216_bfp_read_support(php, bfp, supported_read_modes[idx],
							     &read_instr) < 0) {
					/* not supported */
					continue;
				}

				LOG_DBG("Supports read mode: %d, instr: 0x%X",
					supported_read_modes[idx], read_instr.instr);
				data->read_mode = supported_read_modes[idx];
				data->read_opcode = read_instr.instr;
				data->read_dummy =
					(read_instr.wait_states + read_instr.mode_clocks);
			}
		}

		/* convert 3-Byte opcodes to 4-Byte (if required) */
		if (dev_cfg->four_byte_opcodes) {
			if (data->address_width != 4U) {
				LOG_DBG("4-Byte opcodes require 4-Byte address width");
				return -ENOTSUP;
			}
			data->read_opcode = spi_nor_convert_read_to_4b(data->read_opcode);
			data->write_opcode = spi_nor_convert_write_to_4b(data->write_opcode);
		}

		/* enable quad mode (if required) */
		if (dev_cfg->data_mode == XSPI_QUAD_MODE) {
			if (jesd216_bfp_decode_dw15(php, bfp, &dw15) < 0) {
				/* will use QER from DTS or default (refer to device data) */
				LOG_WRN("Unable to decode QE requirement [DW15]");
			} else {
				/* bypass DTS QER value */
				data->qer_type = dw15.qer;
			}

			LOG_DBG("QE requirement mode: %x", data->qer_type);

			if (stm32_xspi_enable_qe(dev) < 0) {
				LOG_ERR("Failed to enable QUAD mode");
				return -EIO;
			}

			LOG_DBG("QUAD mode enabled");
		}
	}

	data->page_size = jesd216_bfp_page_size(php, bfp);

	LOG_DBG("Page size %u bytes", data->page_size);
	LOG_DBG("Flash size %zu bytes", flash_size);
	LOG_DBG("Using read mode: %d, instr: 0x%X, dummy cycles: %u",
		data->read_mode, data->read_opcode, data->read_dummy);
	LOG_DBG("Using write instr: 0x%X", data->write_opcode);

	return 0;
}

#ifdef CONFIG_FLASH_STM32_XSPI_DMA
static int flash_stm32_xspi_dma_init(DMA_HandleTypeDef *hdma, struct stream *dma_stream)
{
	int ret;
	/*
	 * DMA configuration
	 * Due to use of XSPI HAL API in current driver,
	 * both HAL and Zephyr DMA drivers should be configured.
	 * The required configuration for Zephyr DMA driver should only provide
	 * the minimum information to inform the DMA slot will be in used and
	 * how to route callbacks.
	 */

	if (!device_is_ready(dma_stream->dev)) {
		LOG_ERR("DMA %s device not ready", dma_stream->dev->name);
		return -ENODEV;
	}
	/* Proceed to the minimum Zephyr DMA driver init of the channel */
	dma_stream->cfg.user_data = hdma;
	/* HACK: This field is used to inform driver that it is overridden */
	dma_stream->cfg.linked_channel = STM32_DMA_HAL_OVERRIDE;
	ret = dma_config(dma_stream->dev, dma_stream->channel, &dma_stream->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel %d", dma_stream->channel);
		return ret;
	}

	/* Proceed to the HAL DMA driver init */
	if (dma_stream->cfg.source_data_size != dma_stream->cfg.dest_data_size) {
		LOG_ERR("DMA Source and destination data sizes not aligned");
		return -EINVAL;
	}

#if defined(CONFIG_SOC_SERIES_STM32H7RSX)
	/*
	 * Assume the DMA is HPDMA because GPDMA does not have request line from XSPI.
	 * Allocate source/destination port based on transfer direction:
	 *  - XSPI is only accessible by HPDMA port 1
	 *  - SRAM is only accessible by HPDMA port 0
	 */
	if (table_direction[dma_stream->cfg.channel_direction] == DMA_PERIPH_TO_MEMORY) {
		hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1 |
			DMA_DEST_ALLOCATED_PORT0;
	} else if (table_direction[dma_stream->cfg.channel_direction] == DMA_MEMORY_TO_PERIPH) {
		hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 |
			DMA_DEST_ALLOCATED_PORT1;
	} else {
		LOG_ERR("DMA direction %d is not valid",
			table_direction[dma_stream->cfg.channel_direction]);
		return -EINVAL;
	}
#else /* CONFIG_SOC_SERIES_STM32H7RSX */
	hdma->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 |
		DMA_DEST_ALLOCATED_PORT0;
#endif /* CONFIG_SOC_SERIES_STM32H7RSX */
	hdma->Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD; /* Fixed value */
	hdma->Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD; /* Fixed value */

	hdma->Init.SrcInc = (dma_stream->src_addr_increment)
		? DMA_SINC_INCREMENTED
		: DMA_SINC_FIXED;
	hdma->Init.DestInc = (dma_stream->dst_addr_increment)
		? DMA_DINC_INCREMENTED
		: DMA_DINC_FIXED;
	hdma->Init.SrcBurstLength = 4;
	hdma->Init.DestBurstLength = 4;

	hdma->Init.Priority = table_priority[dma_stream->cfg.channel_priority];
	hdma->Init.Direction = table_direction[dma_stream->cfg.channel_direction];
	hdma->Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
	hdma->Init.Mode = DMA_NORMAL;
	hdma->Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
	hdma->Init.Request = dma_stream->cfg.dma_slot;

	/*
	 * HAL expects a valid DMA channel (not DMAMUX).
	 * The channel is from 0 to 7 because of the STM32_DMA_STREAM_OFFSET
	 * in the dma_stm32 driver
	 */
	hdma->Instance = LL_DMA_GET_CHANNEL_INSTANCE(dma_stream->reg,
						    dma_stream->channel);

	/* Initialize DMA HAL */
	if (HAL_DMA_Init(hdma) != HAL_OK) {
		LOG_ERR("XSPI DMA Init failed");
		return -EIO;
	}

	if (HAL_DMA_ConfigChannelAttributes(hdma, DMA_CHANNEL_NPRIV) != HAL_OK) {
		LOG_ERR("XSPI DMA Init failed");
		return -EIO;
	}

	LOG_DBG("XSPI with DMA transfer");
	return 0;
}
#endif /* CONFIG_FLASH_STM32_XSPI_DMA */


static int flash_stm32_xspi_init(const struct device *dev)
{
	const struct flash_stm32_xspi_config *dev_cfg = dev->config;
	struct flash_stm32_xspi_data *dev_data = dev->data;
	uint32_t ahb_clock_freq;
	uint32_t prescaler = STM32_XSPI_CLOCK_PRESCALER_MIN;
	int ret;

#if defined(CONFIG_STM32_APP_IN_EXT_FLASH) && defined(CONFIG_XIP)
	/* If MemoryMapped then configure skip init
	 * Check clock status first as reading CR register without bus clock doesn't work on N6
	 * If clock is off, then MemoryMapped is off too and we do init
	 */
	if (clock_control_get_status(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				     (clock_control_subsys_t) &dev_cfg->pclken)
				     == CLOCK_CONTROL_STATUS_ON) {
		if (stm32_xspi_is_memorymap(dev)) {
			LOG_DBG("NOR init'd in MemMapped mode");
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
			ret = setup_pages_layout(dev);
			if (ret != 0) {
				LOG_ERR("layout setup failed: %d", ret);
				return -ENODEV;
			}
#endif
			/* Force HAL instance in correct state */
			dev_data->hxspi.State = HAL_XSPI_STATE_BUSY_MEM_MAPPED;
			/* BUG FIX: the fast-path for XIP-booted systems skips the
			 * normal init body, but erase/write still need dev_data->sem
			 * (count=1) to be valid.  Without this, k_sem_take(K_FOREVER)
			 * in xspi_lock_thread() deadlocks → IWDG reset.
			 */
			k_sem_init(&dev_data->sem, 1, 1);
			k_sem_init(&dev_data->sync, 0, 1);
			/* BUG FIX: fast-path skips stm32_xspi_get_flash_params which
			 * normally resolves write_opcode from SFDP/data_mode. Without
			 * this, write_opcode stays at SPI_NOR_WRITEOC_NONE (0xFF) and
			 * flash_stm32_xspi_write sends instruction 0xFF to the flash
			 * instead of the correct PP command → flash silently ignores PP
			 * (WEL stays set, no data programmed).
			 * Erase works because xspi_erase_block uses a hardcoded opcode.
			 */
			if (dev_data->write_opcode == SPI_NOR_WRITEOC_NONE) {
				switch (dev_cfg->data_mode) {
				case XSPI_OCTO_MODE:
					dev_data->write_opcode = SPI_NOR_OCMD_PAGE_PRG;
					break;
				case XSPI_QUAD_MODE:
					dev_data->write_opcode = SPI_NOR_CMD_PP_1_4_4;
					break;
				case XSPI_DUAL_MODE:
					dev_data->write_opcode = SPI_NOR_CMD_PP_1_1_2;
					break;
				default:
					dev_data->write_opcode = SPI_NOR_CMD_PP;
					break;
				}
			}
			return 0;
		}
	}
#endif /* CONFIG_STM32_APP_IN_EXT_FLASH && CONFIG_XIP */

	/* The SPI/DTR is not a valid config of data_mode/data_rate according to the DTS */
	if ((dev_cfg->data_mode != XSPI_OCTO_MODE)
		&& (dev_cfg->data_rate == XSPI_DTR_TRANSFER)) {
		/* already the right config, continue */
		LOG_ERR("XSPI mode SPI|DUAL|QUAD/DTR is not valid");
		return -ENOTSUP;
	}

	/* Signals configuration */
	ret = pinctrl_apply_state(dev_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("XSPI pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t) &dev_cfg->pclken) != 0) {
		LOG_ERR("Could not enable XSPI clock");
		return -EIO;
	}
	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
					(clock_control_subsys_t) &dev_cfg->pclken,
					&ahb_clock_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken)");
		return -EIO;
	}

	if (dev_cfg->has_pclken_ker) {
		/* Kernel clock config for peripheral if any */
		if (clock_control_configure(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			(clock_control_subsys_t) &dev_cfg->pclken_ker, NULL) != 0) {
			LOG_ERR("Could not select XSPI domain clock");
			return -EIO;
		}

		if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			(clock_control_subsys_t) &dev_cfg->pclken_ker,
								&ahb_clock_freq) < 0) {
			LOG_ERR("Failed call clock_control_get_rate(pclken_ker)");
			return -EIO;
		}
	}

	if (dev_cfg->has_pclken_mgr) {
		/* Clock domain corresponding to the IO-Mgr (XSPIM) */
		if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				(clock_control_subsys_t) &dev_cfg->pclken_mgr) != 0) {
			LOG_ERR("Could not enable XSPI Manager clock");
			return -EIO;
		}
	}

	for (; prescaler <= STM32_XSPI_CLOCK_PRESCALER_MAX; prescaler++) {
		uint32_t clk = STM32_XSPI_CLOCK_COMPUTE(ahb_clock_freq, prescaler);

		if (clk <= dev_cfg->max_frequency) {
			break;
		}
	}

	if (prescaler > STM32_XSPI_CLOCK_PRESCALER_MAX) {
		LOG_ERR("XSPI could not find valid prescaler value");
		return -EINVAL;
	}

	/* Initialize XSPI HAL structure completely */
	dev_data->hxspi.Init.ClockPrescaler = prescaler;
	/* The stm32 hal_xspi driver does not reduce DEVSIZE before writing the DCR1 */
	dev_data->hxspi.Init.MemorySize = find_lsb_set(dev_cfg->flash_size) - 2;
#if defined(XSPI_DCR2_WRAPSIZE)
	dev_data->hxspi.Init.WrapSize = HAL_XSPI_WRAP_NOT_SUPPORTED;
#endif /* XSPI_DCR2_WRAPSIZE */
	/* STR mode else Macronix for DTR mode */
	if (dev_cfg->data_rate == XSPI_DTR_TRANSFER) {
		dev_data->hxspi.Init.MemoryType = HAL_XSPI_MEMTYPE_MACRONIX;
		dev_data->hxspi.Init.DelayHoldQuarterCycle = HAL_XSPI_DHQC_ENABLE;
	}

	if (stm32_xspi_is_memorymap(dev)) {
		/* Memory-mapping could have been set by previous application.
		 * Force HAL instance in correct state.
		 */
		dev_data->hxspi.State = HAL_XSPI_STATE_BUSY_MEM_MAPPED;
	}

	if (HAL_XSPI_Init(&dev_data->hxspi) != HAL_OK) {
		LOG_ERR("XSPI Init failed");
		return -EIO;
	}

	LOG_DBG("XSPI Init'd");

#if (defined(HAL_XSPIM_IOPORT_1) || defined(HAL_XSPIM_IOPORT_2)) && \
	!defined(CONFIG_STM32_XSPIM)
	/* XSPI I/O manager init Function */
	XSPIM_CfgTypeDef xspi_mgr_cfg;

	if (dev_data->hxspi.Instance == STM32_XSPI1) {
		xspi_mgr_cfg.IOPort = HAL_XSPIM_IOPORT_1;
	} else if (dev_data->hxspi.Instance == STM32_XSPI2) {
		xspi_mgr_cfg.IOPort = HAL_XSPIM_IOPORT_2;
	}
	xspi_mgr_cfg.nCSOverride = HAL_XSPI_CSSEL_OVR_DISABLED;
	xspi_mgr_cfg.Req2AckTime = 1;

	if (HAL_XSPIM_Config(&dev_data->hxspi, &xspi_mgr_cfg,
		HAL_XSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
		LOG_ERR("XSPI M config failed");
		return -EIO;
	}

#endif /* (HAL_XSPIM_IOPORT_1 || HAL_XSPIM_IOPORT_2) && !(xspim node) */

#if defined(XSPI_DCR1_DLYBYP)
	/* XSPI delay block init Function */
	HAL_XSPI_DLYB_CfgTypeDef xspi_delay_block_cfg = {0};

	(void)HAL_XSPI_DLYB_GetClockPeriod(&dev_data->hxspi, &xspi_delay_block_cfg);
	/*  with DTR, set the PhaseSel/4 (empiric value from stm32Cube) */
	xspi_delay_block_cfg.PhaseSel /= 4;

	if (HAL_XSPI_DLYB_SetConfig(&dev_data->hxspi, &xspi_delay_block_cfg) != HAL_OK) {
		LOG_ERR("XSPI DelayBlock failed");
		return -EIO;
	}

	LOG_DBG("Delay Block Init");
#endif /* XSPI_DCR1_DLYBYP */

#ifdef CONFIG_FLASH_STM32_XSPI_DMA
	/* Configure and enable the DMA channels after XSPI config */
	static DMA_HandleTypeDef hdma_tx;
	static DMA_HandleTypeDef hdma_rx;

	if (flash_stm32_xspi_dma_init(&hdma_tx, &dev_data->dma_tx) != 0) {
		LOG_ERR("XSPI DMA Tx init failed");
		return -EIO;
	}

	/* The dma_tx handle is hold by the dma_stream.cfg.user_data */
	__HAL_LINKDMA(&dev_data->hxspi, hdmatx, hdma_tx);

	if (flash_stm32_xspi_dma_init(&hdma_rx, &dev_data->dma_rx) != 0) {
		LOG_ERR("XSPI DMA Rx init failed");
		return -EIO;
	}

	/* The dma_rx handle is hold by the dma_stream.cfg.user_data */
	__HAL_LINKDMA(&dev_data->hxspi, hdmarx, hdma_rx);

#endif /* CONFIG_USE_STM32_HAL_DMA */
	/* Initialize semaphores */
	k_sem_init(&dev_data->sem, 1, 1);
	k_sem_init(&dev_data->sync, 0, 1);

	/* Run IRQ init */
	dev_cfg->irq_config(dev);

	if (stm32_xspi_is_memorymap(dev)) {
		/* Memory-mapping could have been set by previous application.
		 * Abort to allow following Jedec transactions, it will be
		 * re-enabled afterwards if needed by the application.
		 */
		ret = stm32_xspi_abort(dev);
		if (ret != 0) {
			LOG_ERR("Failed to abort memory-mapped access before Jedec ops");
			return ret;
		}
	}

	/* Reset NOR flash memory : still with the SPI/STR config for the NOR */
	if (stm32_xspi_mem_reset(dev) != 0) {
		LOG_ERR("XSPI reset failed");
		return -EIO;
	}

	LOG_DBG("Reset Mem (SPI/STR)");

	/* Check if memory is ready in the SPI/STR mode */
	if (stm32_xspi_mem_ready(dev,
		XSPI_SPI_MODE, XSPI_STR_TRANSFER) != 0) {
		LOG_ERR("XSPI memory not ready");
		return -EIO;
	}

	LOG_DBG("Mem Ready (SPI/STR)");

#if defined(CONFIG_FLASH_JESD216_API)
	/* Process with the RDID (jedec read ID) instruction at init and fill jedec_id Table */
	ret = stm32_xspi_read_jedec_id(dev);
	if (ret != 0) {
		LOG_ERR("Read ID failed: %d", ret);
		return ret;
	}
#endif /* CONFIG_FLASH_JESD216_API */

	if (stm32_xspi_config_mem(dev) != 0) {
		LOG_ERR("OSPI mode not config'd (%u rate %u)",
			dev_cfg->data_mode, dev_cfg->data_rate);
		return -EIO;
	}

	/* Send the instruction to read the SFDP  */
	const uint8_t decl_nph = 2;
	union {
		/* We only process BFP so use one parameter block */
		uint8_t raw[JESD216_SFDP_SIZE(decl_nph)];
		struct jesd216_sfdp_header sfdp;
	} u;
	const struct jesd216_sfdp_header *hp = &u.sfdp;

	ret = xspi_read_sfdp(dev, 0, u.raw, sizeof(u.raw));
	if (ret != 0) {
		LOG_ERR("SFDP read failed: %d", ret);
		return ret;
	}

	uint32_t magic = jesd216_sfdp_magic(hp);

	if (magic != JESD216_SFDP_MAGIC) {
		LOG_ERR("SFDP magic %08x invalid", magic);
		return -EINVAL;
	}

	LOG_DBG("%s: SFDP v %u.%u AP %x with %u PH", dev->name,
		hp->rev_major, hp->rev_minor, hp->access, 1 + hp->nph);

	const struct jesd216_param_header *php = hp->phdr;
	const struct jesd216_param_header *phpe = php +
						     MIN(decl_nph, 1 + hp->nph);

	while (php != phpe) {
		uint16_t id = jesd216_param_id(php);

		LOG_DBG("PH%u: %04x rev %u.%u: %u DW @ %x",
			(php - hp->phdr), id, php->rev_major, php->rev_minor,
			php->len_dw, jesd216_param_addr(php));

		if (id == JESD216_SFDP_PARAM_ID_BFP) {
			union {
				uint32_t dw[20];
				struct jesd216_bfp bfp;
			} u2;
			const struct jesd216_bfp *bfp = &u2.bfp;

			ret = xspi_read_sfdp(dev, jesd216_param_addr(php),
					     (uint8_t *)u2.dw,
					     MIN(sizeof(uint32_t) * php->len_dw, sizeof(u2.dw)));
			if (ret == 0) {
				ret = spi_nor_process_bfp(dev, php, bfp);
			}

			if (ret != 0) {
				LOG_ERR("SFDP BFP failed: %d", ret);
				break;
			}
		}
		if (id == JESD216_SFDP_PARAM_ID_4B_ADDR_INSTR) {

			if (dev_data->address_width == 4U) {
				/*
				 * Check table 4 byte address instruction table to get supported
				 * erase opcodes when running in 4 byte address mode
				 */
				union {
					uint32_t dw[2];
					struct {
						uint32_t dummy;
						uint8_t type[4];
					} types;
				} u2;
				ret = xspi_read_sfdp(dev, jesd216_param_addr(php),
					     (uint8_t *)u2.dw,
					     MIN(sizeof(uint32_t) * php->len_dw, sizeof(u2.dw)));
				if (ret != 0) {
					break;
				}
				for (uint8_t ei = 0; ei < JESD216_NUM_ERASE_TYPES; ++ei) {
					struct jesd216_erase_type *etp = &dev_data->erase_types[ei];
					const uint8_t cmd = u2.types.type[ei];
					/* 0xff means not supported */
					if (cmd == 0xff) {
						etp->exp = 0;
						etp->cmd = 0;
					} else {
						etp->cmd = cmd;
					};
				}
			}
		}
		++php;
	}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	ret = setup_pages_layout(dev);
	if (ret != 0) {
		LOG_ERR("layout setup failed: %d", ret);
		return -ENODEV;
	}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

	if (dev_cfg->requires_ulbpr) {
		ret = xspi_write_unprotect(dev);
		if (ret != 0) {
			LOG_ERR("write unprotect failed: %d", ret);
			return -ENODEV;
		}
		LOG_DBG("Write Un-protected");
	}

#ifdef CONFIG_STM32_MEMMAP
	ret = stm32_xspi_set_memorymap(dev);
	if (ret != 0) {
		LOG_ERR("Failed to enable memory-mapped mode: %d", ret);
		return ret;
	}
	LOG_INF("Memory-mapped NOR-flash at 0x%lx (0x%x bytes)",
		(long)(dev_cfg->mem_map_based_address),
		dev_cfg->flash_size);
#else
	LOG_INF("NOR external-flash at 0x%lx (0x%x bytes)",
		(long)(dev_cfg->mem_map_based_address),
		dev_cfg->flash_size);
#endif /* CONFIG_STM32_MEMMAP*/
	return 0;
}


#ifdef CONFIG_FLASH_STM32_XSPI_DMA
#define DMA_CHANNEL_CONFIG(node, dir)						\
		DT_DMAS_CELL_BY_NAME(node, dir, channel_config)

#define XSPI_DMA_CHANNEL_INIT(node, dir, dir_cap, src_dev, dest_dev)		\
	.dev = DEVICE_DT_GET(DT_DMAS_CTLR(node)),				\
	.channel = DT_DMAS_CELL_BY_NAME(node, dir, channel),			\
	.reg = (DMA_TypeDef *)DT_REG_ADDR(					\
				   DT_PHANDLE_BY_NAME(node, dmas, dir)),	\
	.cfg = {								\
		.dma_slot = DT_DMAS_CELL_BY_NAME(node, dir, slot),		\
		.channel_direction = STM32_DMA_CONFIG_DIRECTION(		\
					DMA_CHANNEL_CONFIG(node, dir)),	\
		.channel_priority = STM32_DMA_CONFIG_PRIORITY(			\
					DMA_CHANNEL_CONFIG(node, dir)),		\
		.dma_callback = xspi_dma_callback,				\
	},									\
	.src_addr_increment = STM32_DMA_CONFIG_##src_dev##_ADDR_INC(		\
				DMA_CHANNEL_CONFIG(node, dir)),			\
	.dst_addr_increment = STM32_DMA_CONFIG_##dest_dev##_ADDR_INC(		\
				DMA_CHANNEL_CONFIG(node, dir)),

#define XSPI_DMA_CHANNEL(node, dir, DIR, src, dest)				\
	.dma_##dir = {								\
		COND_CODE_1(DT_DMAS_HAS_NAME(node, dir),			\
			(XSPI_DMA_CHANNEL_INIT(node, dir, DIR, src, dest)),	\
			(NULL))							\
		},
#else
#define XSPI_DMA_CHANNEL(node, dir, DIR, src, dest)
#endif /* CONFIG_USE_STM32_HAL_DMA */

#define DT_WRITEOC_PROP_OR(inst, default_value)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, writeoc),					\
		    (_CONCAT(SPI_NOR_CMD_, DT_STRING_TOKEN(DT_DRV_INST(inst), writeoc))),	\
		    ((default_value)))

#define DT_QER_PROP_OR(inst, default_value)							\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, quad_enable_requirements),			\
		    (_CONCAT(JESD216_DW15_QER_VAL_,						\
			     DT_STRING_TOKEN(DT_DRV_INST(inst), quad_enable_requirements))),	\
		    ((default_value)))

#if defined(HAL_XSPIM_IOPORT_1) || defined(HAL_XSPIM_IOPORT_2)
#define DT_XSPI_STM32_MEMORY_SELECT(inst)				\
		.MemorySelect = ((DT_INST_PROP(inst, ncs_line) == 1)	\
					? HAL_XSPI_CSSEL_NCS1		\
					: HAL_XSPI_CSSEL_NCS2),
#else
#define DT_XSPI_STM32_MEMORY_SELECT(inst)	 /* Not used */
#endif /* HAL_XSPIM_IOPORT_1 || HAL_XSPIM_IOPORT_2 */

#if defined(XSPI_DCR1_DLYBYP)
#define DT_XSPI_STM32_DELAY_BLOCK_BYPASS(inst)						\
		.DelayBlockBypass = (DT_PROP(STM32_XSPI_NODE(inst), dlyb_bypass)	\
					? HAL_XSPI_DELAY_BLOCK_BYPASS			\
					: HAL_XSPI_DELAY_BLOCK_ON),
#else
#define DT_XSPI_STM32_DELAY_BLOCK_BYPASS(inst)	/* Not used */
#endif /* XSPI_DCR1_DLYBYP */

#if defined(XSPI_DCR3_MAXTRAN)
#define XSPI_STM32_MAXTRAN	\
		.MaxTran = 0,
#else
#define XSPI_STM32_MAXTRAN	 /* Not used */
#endif /* XSPI_DCR3_MAXTRAN */

#if defined(XSPI_DCR4_REFRESH)
#define XSPI_STM32_REFRESH	\
		.Refresh = 0,	 /* Not used */
#else
#define XSPI_STM32_REFRESH
#endif /* XSPI_DCR4_REFRESH */

#define XSPI_NOR_STM32_INIT(inst)								\
												\
	PINCTRL_DT_DEFINE(STM32_XSPI_NODE(inst));						\
												\
	static void flash_stm32_xspi_irq_config_func_##inst(const struct device *dev)		\
	{											\
		IRQ_CONNECT(DT_IRQN(STM32_XSPI_NODE(inst)),					\
			    DT_IRQ(STM32_XSPI_NODE(inst), priority),				\
			    flash_stm32_xspi_isr, DEVICE_DT_INST_GET(inst), 0);			\
		irq_enable(DT_IRQN(STM32_XSPI_NODE(inst)));					\
	}											\
												\
	static const struct flash_stm32_xspi_config flash_stm32_xspi_cfg_##inst = {		\
		/* Properties of the controller */						\
		.pclken = STM32_CLOCK_INFO_BY_NAME(STM32_XSPI_NODE(inst), xspix),		\
		IF_ENABLED(DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE(inst), xspi_ker),	(		\
			.has_pclken_ker = true,							\
			.pclken_ker = STM32_CLOCK_INFO_BY_NAME(STM32_XSPI_NODE(inst), xspi_ker),\
		))										\
		IF_ENABLED(DT_CLOCKS_HAS_NAME(STM32_XSPI_NODE(inst),  xspi_mgr), (		\
			.has_pclken_mgr = true,							\
			.pclken_mgr = STM32_CLOCK_INFO_BY_NAME(STM32_XSPI_NODE(inst), xspi_mgr),\
		))										\
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(STM32_XSPI_NODE(inst)),			\
		.irq_config = flash_stm32_xspi_irq_config_func_##inst,				\
		.mem_map_based_address = DT_REG_ADDR_BY_IDX(STM32_XSPI_NODE(inst), 1),		\
												\
		/* Properties of the flash device */						\
		.flash_size = DT_INST_PROP(inst, size) / 8,			/* In Bytes */	\
		.max_frequency = DT_INST_PROP(inst, ospi_max_frequency),			\
		.data_mode = DT_INST_PROP(inst, spi_bus_width),			/* SPI or OPI */\
		.data_rate = DT_INST_PROP(inst, data_rate),			/* DTR or STR */\
		.four_byte_opcodes = DT_INST_PROP_OR(inst, four_byte_opcodes, 0),		\
		.requires_ulbpr = DT_INST_PROP_OR(inst, requires_ulbpr, 0),			\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, reset_gpios), (				\
			.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),				\
			.reset_gpios_duration = DT_INST_PROP_OR(inst, reset_gpios_duration, 1),	\
		))										\
	};											\
												\
	static struct flash_stm32_xspi_data flash_stm32_xspi_dev_data_##inst = {		\
		.hxspi = {									\
			.Instance = (XSPI_TypeDef *)DT_REG_ADDR(STM32_XSPI_NODE(inst)),		\
			.Init = {								\
				.FifoThresholdByte = STM32_XSPI_FIFO_THRESHOLD,			\
				.SampleShifting = (DT_PROP(STM32_XSPI_NODE(inst), ssht_enable)	\
						? HAL_XSPI_SAMPLE_SHIFT_HALFCYCLE		\
						: HAL_XSPI_SAMPLE_SHIFT_NONE),			\
				.ChipSelectHighTimeCycle = 2,					\
				.ClockMode = HAL_XSPI_CLOCK_MODE_0,				\
				.ChipSelectBoundary = 0,					\
				.MemoryMode = HAL_XSPI_SINGLE_MEM,				\
				.FreeRunningClock = HAL_XSPI_FREERUNCLK_DISABLE,		\
				DT_XSPI_STM32_MEMORY_SELECT(inst)				\
				DT_XSPI_STM32_DELAY_BLOCK_BYPASS(inst)				\
				XSPI_STM32_MAXTRAN						\
				XSPI_STM32_REFRESH						\
			},									\
		},										\
												\
		.qer_type = DT_QER_PROP_OR(inst, JESD216_DW15_QER_VAL_S1B6),			\
		.write_opcode = DT_WRITEOC_PROP_OR(inst, SPI_NOR_WRITEOC_NONE),			\
		.page_size = SPI_NOR_PAGE_SIZE, /* by default, to be updated by sfdp */		\
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, jedec_id), (				\
			.jedec_id = DT_INST_PROP(inst, jedec_id),				\
		))										\
												\
		XSPI_DMA_CHANNEL(STM32_XSPI_NODE(inst), tx, TX, MEMORY, PERIPHERAL)		\
		XSPI_DMA_CHANNEL(STM32_XSPI_NODE(inst), rx, RX, PERIPHERAL, MEMORY)		\
	};											\
												\
	DEVICE_DT_INST_DEFINE(inst, &flash_stm32_xspi_init, NULL,				\
			      &flash_stm32_xspi_dev_data_##inst, &flash_stm32_xspi_cfg_##inst,	\
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,			\
			      &flash_stm32_xspi_driver_api);

#define XSPI_STM32_INIT(inst)							\
	IF_ENABLED(DT_NODE_HAS_STATUS_OKAY(STM32_XSPI_NODE(inst)),		\
		   (XSPI_NOR_STM32_INIT(inst)))

DT_INST_FOREACH_STATUS_OKAY(XSPI_STM32_INIT)
