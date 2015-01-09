/*
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2013 Benjamin Vernoux <bvernoux@gmail.com>
 *
 * This file is part of AirSpy (based on HackRF project).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <stdint.h>
#include "fault_handler.h"

typedef struct
{
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr; /* Link Register. */
  uint32_t pc; /* Program Counter. */
  uint32_t psr;/* Program Status Register. */
} hard_fault_stack_t;

/* Code Compatible with CortexM0, M3, M4 (R4 & R5 are modified for ARCH 6M) */
__attribute__((naked))
void hard_fault_handler(void)
{
#if defined(__ARM_ARCH_6M__)
	__asm__("MOVS R4, #4");
	__asm__("MOV  R5, LR");
	__asm__("TST R4, R5");
	__asm__("BEQ _MSP");
	__asm__("MRS R0, PSP");
	__asm__("B hard_fault_handler_c");
	__asm__("_MSP:");
	__asm__("MRS R0, MSP");
	__asm__("B hard_fault_handler_c");
#elif defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
  __asm__("TST LR, #4");
  __asm__("ITE EQ");
  __asm__("MRSEQ R0, MSP");
  __asm__("MRSNE R0, PSP");
  __asm__("B hard_fault_handler_c");
#else
  #error "unsupported CortexMx architecture"
#endif
}

volatile hard_fault_stack_t* hard_fault_stack_pt;

__attribute__((used)) void hard_fault_handler_c(uint32_t* args)
{
  /* hard_fault_stack_pt contains registers saved before the hard fault */
  hard_fault_stack_pt = (hard_fault_stack_t*)args;

  /*
  // args[0-7]: r0, r1, r2, r3, r12, lr, pc, psr
  // Other interesting registers to examine (see SCB data structure):
	//	CFSR: Configurable Fault Status Register
	//	HFSR: Hard Fault Status Register
	//	DFSR: Debug Fault Status Register
	//	AFSR: Auxiliary Fault Status Register
	//	MMAR: MemManage Fault Address Register
	//	BFAR: Bus Fault Address Register

  if( SCB->HFSR & SCB_HFSR_FORCED ) {
    if( SCB->CFSR & SCB_CFSR_BFSR_BFARVALID ) {
      SCB->BFAR;
      if( SCB->CFSR & CSCB_CFSR_BFSR_PRECISERR ) {
      }
    }
  }
  */
  __asm("BKPT #0\n") ; // Break into the debugger

  while(1);
}

void mem_manage_handler() {
  while(1);
}

void bus_fault_handler() {
  while(1);
}

void usage_fault_handler() {
  while(1);
}
