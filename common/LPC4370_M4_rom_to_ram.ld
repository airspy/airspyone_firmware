/*
 * This file is part of the libopencm3 project.

 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2013/2014 Benjamin Vernoux <bvernoux@gmail.com>
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

/* Linker script for AirSpy (LPC4370, 1M or 4MB SPI flash, 282K SRAM). */
MEMORY
{
	/* Physical address in Flash used to copy Code from Flash to RAM */
	rom_flash (rx)  : ORIGIN = 0x80000000, LENGTH =  1M
	/* rom is really the shadow region that points to SPI flash or elsewhere */
	rom (rx)  : ORIGIN = 0x00000000, LENGTH =  1M
	ram_local1 (rwx) : ORIGIN = 0x10000000, LENGTH =  128K
	ram_local2 (rwx) : ORIGIN = 0x10080000, LENGTH =  72K
	/* LPC437x only: Reserved for M0 subsystem SRAM 16KB + 2KB  */
	ram_local_m0sub (rwx) : ORIGIN = 0x18000000, LENGTH =  18K

	ram_ahb1_m0_stack (rwx) : ORIGIN = 0x20000000, LENGTH =  4K
	ram_ahb1_spare    (rwx) : ORIGIN = 0x20001000, LENGTH =  3K
	ram_ahb1_m4_share (rwx) : ORIGIN = 0x20001C00, LENGTH =  1K
	ram_ahb1_adchs (rwx)    : ORIGIN = 0x20002000, LENGTH =  4K
	ram_ahb1_m0_share (rwx) : ORIGIN = 0x20003000, LENGTH =  4K

	/* Removed 32K of AHB SRAM for USB buffer. Straddles two blocks of RAM
	 * to get performance benefit of having two USB buffers addressable
	 * simultaneously (on two different buses of the AHB multilayer matrix)
	 */
	/* ram_ahb1_0 (rwx) : ORIGIN = 0x20004000, LENGTH =  16K */
	/* ram_ahb1_1 (rwx) : ORIGIN = 0x20008000, LENGTH =  16K */
	/* Reserved for Cortex M0 code/data */
	ram_ahb2   (rwx) : ORIGIN = 0x2000C000, LENGTH =  16K
}

cm4_data_share = ORIGIN(ram_ahb1_m4_share);	/* M4 data shared (write by M4 read by M0 */
adchs_data     = ORIGIN(ram_ahb1_adchs);	/* M4 ADCHS data (write by ADCHS DMA or M4) */
cm0_data_share = ORIGIN(ram_ahb1_m0_share);	/* M0 data shared (write by M0 read by M4) */

cm0_exec_baseaddr = ORIGIN(ram_ahb2);	/* Used by M4 to load code from M4 RAM to M0 final exec/run addr */
cm0s_exec_baseaddr = ORIGIN(ram_local_m0sub);	/* Used by M4 to load code from M4 RAM to M0Sub final exec/run addr */

/* Include the common ld script. */
INCLUDE libopencm3_lpc43xx_rom_to_ram.ld
