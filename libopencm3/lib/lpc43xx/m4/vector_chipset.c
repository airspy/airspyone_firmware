/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2012 Michael Ossmann <mike@ossmann.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/scb.h>

extern unsigned _etext_ram, _text_ram, _etext_rom;

#define CREG_M4MEMMAP   MMIO32((0x40043000 + 0x100))

static void pre_main(void)
{
	volatile unsigned *src, *dest;

  // *************************************************************
  // The following conditional block of code manually resets as
  // much of the peripheral set of the LPC43 as possible. This is
  // done because the LPC43 does not provide a means of triggering
  // a full system reset under debugger control, which can cause
  // problems in certain circumstances when debugging.
  //
  // You can prevent this code block being included if you require
  // (for example when creating a final executable which you will
  // not debug) by setting the define 'DONT_RESET_ON_RESTART'.
  //
  #ifndef DONT_RESET_ON_RESTART
  // Disable interrupts
  __asm volatile ("cpsid i");
  // equivalent to CMSIS '__disable_irq()' function
  unsigned int *RESET_CONTROL = (unsigned int *) 0x40053100;
  // LPC_RGU->RESET_CTRL0 @ 0x40053100
  // LPC_RGU->RESET_CTRL1 @ 0x40053104
  // Note that we do not use the CMSIS register access mechanism,
  // as there is no guarantee that the project has been configured
  // to use CMSIS.
  // Write to LPC_RGU->RESET_CTRL0
  *(RESET_CONTROL+0) = 0x10DF1000;
  // GPIO_RST|AES_RST|ETHERNET_RST|SDIO_RST|DMA_RST|
  // USB1_RST|USB0_RST|LCD_RST|M0_SUB_RST
  // Write to LPC_RGU->RESET_CTRL1
  *(RESET_CONTROL+1) = 0x01DFF7FF;
  // M0APP_RST|CAN0_RST|CAN1_RST|I2S_RST|SSP1_RST|SSP0_RST|
  // I2C1_RST|I2C0_RST|UART3_RST|UART1_RST|UART1_RST|UART0_RST|
  // DAC_RST|ADC1_RST|ADC0_RST|QEI_RST|MOTOCONPWM_RST|SCT_RST|
  // RITIMER_RST|TIMER3_RST|TIMER2_RST|TIMER1_RST|TIMER0_RST
  // Clear all pending interrupts in the NVIC
  volatile unsigned int *NVIC_ICPR = (unsigned int *) 0xE000E280;
  unsigned int irqpendloop;
  for(irqpendloop = 0; irqpendloop < 8; irqpendloop++)
  {
    *(NVIC_ICPR+irqpendloop)= 0xFFFFFFFF;
  }
  // Reenable interrupts
  __asm volatile ("cpsie i");
  // equivalent to CMSIS '__enable_irq()' function
  #endif // ifndef DONT_RESET_ON_RESTART
  // *************************************************************

	/* Copy the code from ROM to Real RAM (if enabled) */
	if ((&_etext_ram-&_text_ram) > 0) {
		src = &_etext_rom-(&_etext_ram-&_text_ram);
		/* Change Shadow memory to ROM (for Debug Purpose in case Boot
		 * has not set correctly the M4MEMMAP because of debug)
		 */
		CREG_M4MEMMAP = (unsigned long)src;

		for (dest = &_text_ram; dest < &_etext_ram; ) {
			*dest++ = *src++;
		}

		/* Change Shadow memory to Real RAM */
		CREG_M4MEMMAP = (unsigned long)&_text_ram;

		/* Continue Execution in RAM */
	}

	/* Enable access to Floating-Point coprocessor. */
	SCB_CPACR |= SCB_CPACR_FULL * (SCB_CPACR_CP10 | SCB_CPACR_CP11);
}
