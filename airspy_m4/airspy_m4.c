/*
 * Copyright 2013/2014 Benjamin Vernoux <bvernoux@gmail.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
 *
 * This file is part of AirSpy.
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

#include <string.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/ipc.h>
#include <libopencm3/cm3/scs.h>

#include <airspy_core.h>
#include <si5351c.h>
#include <w25q80bv.h>
#include <rom_iap.h>
#include <signal_mcu.h>

#include "adchs.h"

#include "m0_bin.h"
#include "m0s_bin.h"

#include "airspy_conf.h"

//extern volatile uint32_t *usb_bulk_buffer_offset;

#define DEFAULT_ADCHS_CHAN (0)

#undef DMA_ISR_DEBUG
//#define DMA_ISR_DEBUG
/*
#define USB_DATA_TRANSFER_SIZE_BYTE (ADCHS_DATA_TRANSFER_SIZE_BYTE)
#ifdef USE_PACKING
#define USB_BULK_BUFFER_MASK ((4) - 1)
#else
#define USB_BULK_BUFFER_MASK ((32768) - 1)
#endif
#define get_usb_buffer_offset() (usb_bulk_buffer_offset[0])
#define set_usb_buffer_offset(val) (usb_bulk_buffer_offset[0] = val)
// Manage round robin after increment with USB_BULK_BUFFER_MASK
#define inc_mask_usb_buffer_offset(buff_offset, inc_value) ((buff_offset+inc_value) & USB_BULK_BUFFER_MASK)

#ifdef USE_PACKING
volatile uint32_t usb_bulk_buffer_offset_uint32_m4;
volatile uint32_t *usb_bulk_buffer_offset_m4;
#define get_usb_buffer_offset_m4() (usb_bulk_buffer_offset_m4[0])
#define set_usb_buffer_offset_m4(val) (usb_bulk_buffer_offset_m4[0] = val)
#define inc_mask_usb_buffer_offset_m4(buff_offset, inc_value) inc_mask_usb_buffer_offset(buff_offset, inc_value)
volatile unsigned int phase = 0;
#endif
*/

volatile uint32_t adchs_dma_buffer_offset_isr_m4  = 0;
volatile uint32_t adchs_dma_buffer_offset_main_m4 = 0;
volatile uint32_t usb_dma_buffer_length_main_m4 = 0;
volatile uint32_t usb_dma_buffer_offset_main_m4 = 0;

#define SLAVE_TXEV_FLAG ((uint32_t *) 0x40043400)
#define SLAVE_TXEV_QUIT() { *SLAVE_TXEV_FLAG = 0x0; }

extern uint32_t cm4_data_share; /* defined in linker script */
extern uint32_t adchs_data; /* defined in linker script */
extern uint32_t cm0_data_share; /* defined in linker script */

volatile int adchs_stopped = 0;
volatile int adchs_started = 0;

//volatile uint32_t *usb_bulk_buffer_offset = &cm4_data_share;
volatile airspy_m0_queue_t *m0_queue = (void *) (&cm4_data_share);
uint8_t* const usb_bulk_buffer = (uint8_t*) USB_BULK_BUFFER_START;
uint8_t* const adchs_dma_buffer = (uint8_t*) ADCHS_DATA_BUFFER;

volatile uint32_t *start_adchs = (&cm0_data_share);
#define START_ADCHS_CMD  (1)
#define STOP_ADCHS_CMD   (2)

volatile uint32_t *set_samplerate = ((&cm0_data_share)+1);
#define SET_SAMPLERATE_CMD  (1)

#define CMD_MASK ((1 << 2)-1)
#define CONF_MASK ((1 << AIRSPY_SAMPLERATE_CMD_SHIFT_BIT)-1)

volatile int first_start = 0;

#define AIRSPY_FORMAT_MAX (2)
typedef enum
{
	AIRSPY_FORMAT_PACK16       = 0,
	AIRSPY_FORMAT_PACK12       = 1,
	AIRSPY_FORMAT_PACK8        = AIRSPY_FORMAT_MAX
} airspy_format_t;
volatile airspy_format_t airspy_format = AIRSPY_FORMAT_PACK16;

/*
uint32_t nb_cycles[5] = { 0 };
uint32_t data_counter = 0;
*/
#ifdef DMA_ISR_DEBUG
  #define DMA_IRQ_CYCLES_MAX (100)
  #define FREQ_DMA_IRQ_CYCLES_MAX (100)
  typedef struct
  {
    uint32_t adchs_fifo_ovf;
    uint32_t adchs_dscr_error;
    uint32_t adchs_adc_ovf;
    uint32_t adchs_adc_unf;
    uint32_t dma_err_cnt;

    uint32_t dma_irq_cycles[DMA_IRQ_CYCLES_MAX];
    uint32_t dma_irq_cycles_idx;

    uint32_t freq_dma_irq_cycles[FREQ_DMA_IRQ_CYCLES_MAX];
    uint32_t freq_dma_irq_cycles_idx;
  } t_stats_adchs;

  t_stats_adchs stat_adchs = { 0 };
#endif
//
// Packing Works @40MSPS with CPU Clk = 140Mhz : USB Bandwidth 20MBytes/sec
// USB Bandwidth limits output to 20MBytse/sec = 20MSPS
//
__attribute__ ((always_inline)) static uint32_t pack16(uint8_t* input, uint8_t* output, uint32_t length) {
  uint32_t i;
  uint32_t* pIn  = (uint32_t*) input;
  uint32_t* pOut = (uint32_t*) output;
  uint32_t  iLen = length >> 4;
    
  for (i = 0; i < iLen; i++) {
    *pOut++ = *pIn++; // 0bbb0aaa
    *pOut++ = *pIn++; // 0ddd0ccc
    *pOut++ = *pIn++; // 0fff0eee
    *pOut++ = *pIn++; // 0hhh0ggg
  }

  return (iLen << 4);
}
//
// Packing Works @24MSPS with CPU Clk = 140Mhz : USB Bandwidth 18MBytes/sec
// USB Bandwidth limits output to 20MBytse/sec = 26.666MSPS
//
__attribute__ ((always_inline)) static uint32_t pack12(uint8_t* input, uint8_t* output, uint32_t length) {
  uint32_t i;
  uint32_t* pIn  = (uint32_t*) input;
  uint32_t* pOut = (uint32_t*) output;
  uint32_t  iLen = length >> 5;
    
  for (i = 0; i < iLen; i++) {    
    register uint32_t t0, t1, t2, t3, q0, q1, q2;
    
    t0 = *pIn++; // t0 = 0bbb0aaa
    t1 = *pIn++; // t1 = 0ddd0ccc
    t2 = *pIn++; // t2 = 0fff0eee
    t3 = *pIn++; // t3 = 0hhh0ggg

    q0 = ((t1 << 24) & 0xFF000000) | ((t0 >>  4) & 0x00FFF000) |  (t0        & 0x00000FFF);                            // q0 = ccbbbaaa
    q1 = ((t2 << 12) & 0xF0000000) | ((t2 << 16) & 0x0FFF0000) | ((t1 >> 12) & 0x0000FFF0) | ((t1 >> 4) & 0x0000000F); // q1 = feeedddc
    q2 = ((t3 <<  4) & 0xFFF00000) | ((t3 <<  8) & 0x000FFF00) | ((t2 >> 20) & 0x000000FF);                            // q2 = hhhgggff       

    *pOut++ = q0; // ccbbbaaa
    *pOut++ = q1; // feeedddc
    *pOut++ = q2; // hhhgggff

    t0 = *pIn++; // t0 = 0bbb0aaa
    t1 = *pIn++; // t1 = 0ddd0ccc
    t2 = *pIn++; // t2 = 0fff0eee
    t3 = *pIn++; // t3 = 0hhh0ggg

    q0 = ((t1 << 24) & 0xFF000000) | ((t0 >>  4) & 0x00FFF000) |  (t0        & 0x00000FFF);                            // q0 = ccbbbaaa
    q1 = ((t2 << 12) & 0xF0000000) | ((t2 << 16) & 0x0FFF0000) | ((t1 >> 12) & 0x0000FFF0) | ((t1 >> 4) & 0x0000000F); // q1 = feeedddc
    q2 = ((t3 <<  4) & 0xFFF00000) | ((t3 <<  8) & 0x000FFF00) | ((t2 >> 20) & 0x000000FF);                            // q2 = hhhgggff       

    *pOut++ = q0; // ccbbbaaa
    *pOut++ = q1; // feeedddc
    *pOut++ = q2; // hhhgggff

  }
  return (iLen * 24);
}
//
// Packing Works @32MSPS with CPU Clk = 140Mhz : USB Bandwidth 16MBytes/sec
// Packing Works @40MSPS with CPU Clk = 160Mhz : USB Bandwidth 20MBytes/sec
//
__attribute__ ((always_inline)) static uint32_t pack8(uint8_t* input, uint8_t* output, uint32_t length) {
  uint32_t i;
  uint32_t* pIn  = (uint32_t*) input;
  uint32_t* pOut = (uint32_t*) output;
  uint32_t  iLen = length >> 5;

  for (i = 0; i < iLen; i++) {    
    register uint32_t t0, t1, t2, t3, tmp, q0, q1;
       
    t0 = *pIn++; // t0 = 0bbb0aaa
    t1 = *pIn++; // t1 = 0ddd0ccc
    t2 = *pIn++; // t2 = 0fff0eee
    t3 = *pIn++; // t3 = 0hhh0ggg

    q0  = 0xFF & (t0  >>  4); // q0  = 000000aa
    tmp = 0xFF & (t0  >> 20); // tmp = 000000bb
    q0  = q0   | (tmp <<  8); // q0  = 0000bbaa
    tmp = 0xFF & (t1  >>  4); // tmp = 000000cc
    q0  = q0   | (tmp << 16); // q0  = 00ccbbaa
    tmp = 0xFF & (t1  >> 20); // tmp = 000000dd
    q0  = q0   | (tmp << 24); // q0  = ddccbbaa
  
    q1  = 0xFF & (t2  >>  4); // q1  = 000000ee
    tmp = 0xFF & (t2  >> 20); // tmp = 000000ff
    q1  = q1   | (tmp <<  8); // q1  = 0000ffee
    tmp = 0xFF & (t3  >>  4); // tmp = 000000gg
    q1  = q1   | (tmp << 16); // q1  = 00ggffee
    tmp = 0xFF & (t3  >> 20); // tmp = 000000hh
    q1  = q1   | (tmp << 24); // q1  = hhggffee

    *pOut++ = q0; // ddccbbaa
    *pOut++ = q1; // hhggffee
       
    t0 = *pIn++; // t0 = 0bbb0aaa
    t1 = *pIn++; // t1 = 0ddd0ccc
    t2 = *pIn++; // t2 = 0fff0eee
    t3 = *pIn++; // t3 = 0hhh0ggg

    q0  = 0xFF & (t0  >>  4); // q0  = 000000aa
    tmp = 0xFF & (t0  >> 20); // tmp = 000000bb
    q0  = q0   | (tmp <<  8); // q0  = 0000bbaa
    tmp = 0xFF & (t1  >>  4); // tmp = 000000cc
    q0  = q0   | (tmp << 16); // q0  = 00ccbbaa
    tmp = 0xFF & (t1  >> 20); // tmp = 000000dd
    q0  = q0   | (tmp << 24); // q0  = ddccbbaa
  
    q1  = 0xFF & (t2  >>  4); // q1  = 000000ee
    tmp = 0xFF & (t2  >> 20); // tmp = 000000ff
    q1  = q1   | (tmp <<  8); // q1  = 0000ffee
    tmp = 0xFF & (t3  >>  4); // tmp = 000000gg
    q1  = q1   | (tmp << 16); // q1  = 00ggffee
    tmp = 0xFF & (t3  >> 20); // tmp = 000000hh
    q1  = q1   | (tmp << 24); // q1  = hhggffee

    *pOut++ = q0; // ddccbbaa
    *pOut++ = q1; // hhggffee
  }
  return (iLen << 4);
}
/*
//
__attribute__ ((always_inline)) static uint32_t pack16(uint8_t* input, uint8_t* output, uint32_t length) {
  register uint32_t* n4 asm("r0") = (uint32_t*) input;
  register uint32_t* n5 asm("r1") = (uint32_t*) output;
  register uint32_t  n6 asm("r2") = length;
  register uint32_t  n7 asm("r8");
  asm volatile (
       "lsr       %2,%2,#4\n\t"         //r2 = length >> 4 ; copy 16 bytes / pack 8 samples per loop 
       "mov       r8,%2\n\t"            //r8 = iLen
       "1:\n\t"
       "ldmia.w   %0,{r3,r4,r5,r6}\n\t" //r3 = 0bbb0aaa, r4 = 0ddd0ccc, r5 = 0fff0eee, r6 = 0hhh0ggg
       "add       %0,%0,#16\n\t"
       "stmia.w   %1,{r3,r4,r5,r6}\n\t"
       "add       %1,%1,#16\n\t"

       "subs      %2,%2,#1\n\t"
       "bne       1b\n\t"
       : "+r" (n4), "+r" (n5), "+r" (n6), "+r" (n7) :: "r3", "r4", "r5", "r6", "cc", "memory");

  return (n7 << 4);
}
//
__attribute__ ((always_inline)) static uint32_t pack12(uint8_t* input, uint8_t* output, uint32_t length) {
  register uint32_t* n4 asm("r0") = (uint32_t*) input;
  register uint32_t* n5 asm("r1") = (uint32_t*) output;
  register uint32_t  n6 asm("r2") = length;
  register uint32_t  n7 asm("r8");
  asm volatile (
       "lsr       %2,%2,#4\n\t"          //r2 = nLen >> 4 ; copy 16 bytes / pack 8 samples per loop
       "mov       r8,%2\n\t"             //r8 = iLen
       "1:\n\t"
       "ldmia.w   %0,{r3,r4,r5,r6}\n\t"  //r3 = 0bbb0aaa, r4 = 0ddd0ccc, r5 = 0fff0eee, r6 = 0hhh0ggg
       "add       %0,%0,#16\n\t"

       "lsr       r7,r3,#4\n\t"          //r7 = 00bbb0aa
       "bfi       r3,r7,#12,#12\n\t"     //r3 = 00bbbaaa
       "orr       r3,r3,r4,lsl #24\n\t"  //r3 = ccbbbaaa

       "lsr       r4,r4,#8\n\t"          //r4 = 000ddd0c
       "lsr       r7,r4,#4\n\t"          //r7 = 0000ddd0
       "bfi       r4,r7,#4,#12\n\t"      //r4 = 0000dddc
       "orr       r4,r4,r5,lsr #16\n\t"  //r4 = 0eeedddc
       "ror       r5,r5,#20\n\t"         //r5 = f0eee0ff
       "bfi       r4,r5,#28,#4\n\t"      //r4 = feeedddc
 
       "ror       r6,r6,#24\n\t"         //r6 = hh0ggg0h
       "bfi       r5,r6,#8,#12\n\t"      //r5 = 000gggff
       "ror       r6,r6,#4\n\t"          //r6 = hhh0ggg0
       "bfi       r5,r6,#20,#12\n\t"     //r5 = hhhgggff

       "stmia.w   %1,{r3,r4,r5}\n\t"
       "add       %1,%1,#12\n\t"

       "subs      %2,%2,#1\n\t"
       "bne       1b"         
       : "+r" (n4), "+r" (n5), "+r" (n6), "+r" (n7) :: "r3", "r4", "r5", "r6", "r7", "cc", "memory");

  return (n7 * 12);
}
//
__attribute__ ((always_inline)) static uint32_t pack8(uint8_t* input, uint8_t* output, uint32_t length) {
  register uint32_t* n4 asm("r0") = (uint32_t*) input;
  register uint32_t* n5 asm("r1") = (uint32_t*) output;
  register uint32_t  n6 asm("r2") = length;
  register uint32_t  n7 asm("r8");

  asm volatile (
       "lsr       %2,%2,#4\n\t"         //r2 = nLen >> 4 ; copy 16 bytes / pack 8 samples per loop
       "mov       r8,%2\n\t"            //r8 = iLen
       "1:\n\t"
       "ldmia.w   %0,{r3,r4,r5,r6}\n\t" //r3 = 0bbb0aaa, r4 = 0ddd0ccc, r5 = 0fff0eee, r6 = 0hhh0ggg
       "add       %0,%0,#16\n\t"

       "ubfx      r7,r3,#20,#8\n\t"     //r7 = 000000bb
       "ubfx      r3,r3,#4,#8\n\t"      //r3 = 000000aa
       "orr       r3,r3,r7,lsl #8\n\t"  //r3 = 0000bbaa 
       "ubfx      r7,r4,#4,#8\n\t"      //r7 = 000000cc
       "orr       r3,r3,r7,lsl #16\n\t" //r3 = 00ccbbaa 
       "ubfx      r7,r4,#20,#8\n\t"     //r7 = 000000dd
       "orr       r3,r3,r7,lsl #24\n\t" //r3 = ddccbbaa 

       "ubfx      r7,r5,#20,#8\n\t"     //r7 = 000000ff
       "ubfx      r4,r5,#4,#8\n\t"      //r4 = 000000ee
       "orr       r4,r4,r7,lsl #8\n\t"  //r4 = 0000ffee 
       "ubfx      r7,r6,#4,#8\n\t"      //r7 = 000000gg
       "orr       r4,r4,r7,lsl #16\n\t" //r4 = 00ggffee 
       "ubfx      r7,r6,#20,#8\n\t"     //r7 = 000000hh
       "orr       r4,r4,r7,lsl #24\n\t" //r4 = hhggffee 

       "stmia.w   %1,{r3,r4}\n\t"
       "add       %1,%1,#8\n\t"

       "subs      %2,%2,#1\n\t"
       "bne       1b"
       : "+r" (n4), "+r" (n5), "+r" (n6), "+r" (n7) :: "r3", "r4", "r5", "r6", "r7", "cc", "memory");

  return (n7 << 3);
}
//
*/
static __inline__ void clr_usb_buffer_offset(void)
{
  adchs_dma_buffer_offset_isr_m4 = 0;
  adchs_dma_buffer_offset_main_m4 = 0;
  usb_dma_buffer_length_main_m4 = 0;
  usb_dma_buffer_offset_main_m4 = 0;

  m0_queue->q_head = 0;
  m0_queue->q_tail = 0;
/*
#ifdef USE_PACKING 
  usb_bulk_buffer_offset[0] = 3; 
  usb_bulk_buffer_offset_m4[0] = 3;
#else
  usb_bulk_buffer_offset[0] = 0;
#endif
*/
}

static __inline__ uint32_t get_start_stop_adchs(void)
{
  uint32_t val;
  uint32_t cmd;

  val = start_adchs[0];
  cmd = (val & CMD_MASK);

  return(cmd);
}

/* Acknowledge Start/Stop ADCHS by clearing the data */
static __inline__ void ack_start_stop_adchs(void)
{
  start_adchs[0] = 0;
}

static __inline__ uint32_t get_samplerate(uint32_t *conf_number)
{
  uint32_t val;
  uint32_t cmd;

  val = set_samplerate[0];
  cmd = (val & CMD_MASK);
  *conf_number = ((val >> AIRSPY_SAMPLERATE_CMD_SHIFT_BIT) & CONF_MASK);

  return(cmd);
}

/* Acknowledge set_samplerate by clearing the data */
static __inline__ void ack_samplerate(void)
{
  set_samplerate[0] = 0;
}

void adchs_start(uint8_t chan_num)
{
  int i;
  uint32_t *dst;

  /* Disable IRQ globally */
  __asm__("cpsid i");

//  cpu_clock_pll1_high_speed(&airspy_m4_init_conf.pll1_hs);
  if(first_start == 0)
  {
    cpu_clock_pll1_high_speed(&airspy_m4_init_conf.pll1_hs);
    first_start = 1;
  }

  /* Clear ADCHS Buffer */
  dst = (uint32_t *)ADCHS_DATA_BUFFER;
  for(i=0; i<(ADCHS_DATA_BUFFER_SIZE_BYTE/4); i++)
  {
    dst[i] = 0;
  }
  clr_usb_buffer_offset();

  ADCHS_init();
  ADCHS_desc_init(chan_num);
  ADCHS_DMA_init((uint32_t)ADCHS_DATA_BUFFER);

  led_on();
  LPC_ADCHS->TRIGGER = 1;
  __asm("dsb");

  /* Enable IRQ globally */
  __asm__("cpsie i");
}

void adchs_stop(void)
{
  /* Disable IRQ globally */
  __asm__("cpsid i");

  ADCHS_deinit();

//  cpu_clock_pll1_low_speed(&airspy_m4_init_conf.pll1_ls);

  led_off();

  /* Enable IRQ globally */
  __asm__("cpsie i");
}

void dma_isr(void) 
{
  uint32_t status;
//#define INTTC0  (1)
  #define ADCHS_DMA_INT_CH  (1)

#ifdef DMA_ISR_DEBUG
  volatile uint32_t tmp_cycles;
  tmp_cycles = SCS_DWT_CYCCNT;

  stat_adchs.freq_dma_irq_cycles[stat_adchs.freq_dma_irq_cycles_idx] = tmp_cycles;
  stat_adchs.freq_dma_irq_cycles_idx++;
  if(stat_adchs.freq_dma_irq_cycles_idx == FREQ_DMA_IRQ_CYCLES_MAX)
    stat_adchs.freq_dma_irq_cycles_idx = 0;

  status = LPC_ADCHS->STATUS0;
  LPC_ADCHS->CLR_STAT0 = status;

  // ADCHS Error stat0

  /* FIFO was full; conversion sample is not stored and lost */
  if(status & STAT0_FIFO_OVERFLOW) 
    stat_adchs.adchs_fifo_ovf++;

  /* The ADC was not fully woken up when a sample was
     converted and the conversion results is unreliable */
  if(status & STAT0_DSCR_ERROR)
    stat_adchs.adchs_dscr_error++;

  /* Converted sample value was over range of the 12 bit output code. */
  if(status & STAT0_ADC_OVF)
    stat_adchs.adchs_adc_ovf++;

  /* Converted sample value was under range of the 12 bit output code. */
  if(status & STAT0_ADC_UNF)
    stat_adchs.adchs_adc_unf++;

  // DMA Error
  status = LPC_GPDMA->INTERRSTAT;
  if( status )
  {
    stat_adchs.dma_err_cnt++; // Count DMA Error
    LPC_GPDMA->INTERRCLR |= status;
  } 
#endif

  status = LPC_GPDMA->INTTCSTAT;
  if (status & ADCHS_DMA_INT_CH)
  {
    LPC_GPDMA->INTTCCLEAR = ADCHS_DMA_INT_CH; /* Clear Chan0 */

    adchs_dma_buffer_offset_isr_m4 = ((adchs_dma_buffer_offset_isr_m4 + ADCHS_DATA_TRANSFER_SIZE_BYTE) & ADCHS_DATA_BUFFER_SIZE_MASK); 
/*
#ifdef USE_PACKING
    set_usb_buffer_offset_m4( inc_mask_usb_buffer_offset_m4(get_usb_buffer_offset_m4(), 1) );    
#else
    set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), USB_DATA_TRANSFER_SIZE_BYTE) );
    signal_sev();
#endif
*/
  }

#ifdef DMA_ISR_DEBUG
  stat_adchs.dma_irq_cycles[stat_adchs.dma_irq_cycles_idx] = (SCS_DWT_CYCCNT - tmp_cycles);
  stat_adchs.dma_irq_cycles_idx++;
  if(stat_adchs.dma_irq_cycles_idx == DMA_IRQ_CYCLES_MAX)
    stat_adchs.dma_irq_cycles_idx = 0;
#endif
}

void m0core_isr(void)
{
  uint32_t adchs_conf;
  uint32_t adchs_start_stop_cmd;
  uint32_t samplerate_cmd;

  SLAVE_TXEV_QUIT();

  samplerate_cmd = get_samplerate(&adchs_conf);
  if(samplerate_cmd == SET_SAMPLERATE_CMD)
  {
    sys_clock_samplerate(&airspy_m4_conf[adchs_conf]);
    ack_samplerate();
  }

  adchs_start_stop_cmd = get_start_stop_adchs();
  switch(adchs_start_stop_cmd)
  {
    case START_ADCHS_CMD:
      if(adchs_started == 0)
      {
        adchs_start(DEFAULT_ADCHS_CHAN);
        adchs_started = 1;
        adchs_stopped = 0;
      }
      ack_start_stop_adchs();
    break;

    case STOP_ADCHS_CMD:
      if(adchs_stopped == 0)
      {
        adchs_stop();
        adchs_stopped = 1;
        adchs_started = 0;
      }
      ack_start_stop_adchs();
    break;

    default:
    /* Invalid command do nothing */
    break;
  }
}

void m0_startup(void)
{
  uint32_t *src, *dest;

  /* Halt M0 core (in case it was running) */
  ipc_halt_m0();

  /* Copy M0 code from M4 embedded addr to final addr M0 */
  dest = &cm0_exec_baseaddr;
  for(src = (uint32_t *)&m0_bin[0]; src < (uint32_t *)(&m0_bin[0]+m0_bin_size); )
  {
    *dest++ = *src++;
  }

  ipc_start_m0( (uint32_t)(&cm0_exec_baseaddr) );
}

void m0s_startup(void)
{
  uint32_t *src, *dest;

  /* Halt M0 core (in case it was running) */
  ipc_halt_m0s();

  /* Copy M0 code from M4 embedded addr to final addr M0 */
  dest = &cm0s_exec_baseaddr;
  for(src = (uint32_t *)&m0s_bin[0]; src < (uint32_t *)(&m0s_bin[0]+m0s_bin_size); )
  {
    *dest++ = *src++;
  }

  ipc_start_m0s( (uint32_t)(&cm0s_exec_baseaddr) );
}

void scs_dwt_cycle_counter_enabled(void)
{
  SCS_DEMCR |= SCS_DEMCR_TRCENA;
  SCS_DWT_CTRL  |= SCS_DWT_CTRL_CYCCNTENA;
}

int main(void)
{
  scs_dwt_cycle_counter_enabled();
  pin_setup();

  sys_clock_init(&airspy_m4_init_conf);

  nvic_set_priority(NVIC_DMA_IRQ, 255);
  nvic_set_priority(NVIC_M0CORE_IRQ, 1);

  clr_usb_buffer_offset();

  nvic_enable_irq(NVIC_DMA_IRQ);
  nvic_enable_irq(NVIC_M0CORE_IRQ);

  adchs_stop();
  adchs_stopped = 1;
  adchs_started = 0;

  ack_start_stop_adchs();
  ack_samplerate();

  /* Start M0 */
  m0_startup();

#undef ENABLE_M0S
#ifdef ENABLE_M0S
  /* Start M0s */
  m0s_startup();
#else
  // Halt M0s
  ipc_halt_m0s();
  // Disable M0 Sub
  CCU1_CLK_PERIPH_CORE_CFG &= ~(1);
#endif

  while(true)
  {
    uint32_t adchs_buffer_offset, usb_buffer_offset, usb_buffer_length, q_head, q_word;
    signal_wfe();
    // If our local offset is different to the isr offset we have new samples to process
    if ((adchs_buffer_offset = adchs_dma_buffer_offset_main_m4) != adchs_dma_buffer_offset_isr_m4)
    {
      // Update our local pointer to the input data
      adchs_dma_buffer_offset_main_m4 = ((adchs_buffer_offset + ADCHS_DATA_TRANSFER_SIZE_BYTE) & ADCHS_DATA_BUFFER_SIZE_MASK);

      // get the pointers to the output buffer
      usb_buffer_length = usb_dma_buffer_length_main_m4;
      usb_buffer_offset = usb_dma_buffer_offset_main_m4 + usb_buffer_length;

      q_word = 0;

      // Pack the ADC samples in the user selected manner. Default to AIRSPY_FORMAT_PACK16
      switch (airspy_format) 
      {
        case AIRSPY_FORMAT_PACK8:
        {
          usb_buffer_length += pack8(&adchs_dma_buffer[adchs_buffer_offset], &usb_bulk_buffer[usb_buffer_offset], ADCHS_DATA_TRANSFER_SIZE_BYTE);
          break;
        }
        case AIRSPY_FORMAT_PACK12:
        {
          usb_buffer_length += pack12(&adchs_dma_buffer[adchs_buffer_offset], &usb_bulk_buffer[usb_buffer_offset], ADCHS_DATA_TRANSFER_SIZE_BYTE);
          break;
        }
        case AIRSPY_FORMAT_PACK16:
        default: 
        {
          usb_buffer_length += pack16(&adchs_dma_buffer[adchs_buffer_offset], &usb_bulk_buffer[usb_buffer_offset], ADCHS_DATA_TRANSFER_SIZE_BYTE);
          break;
        }
      }

      // If the output buffer is > than one-quarter full (but <= half-full) ...
      // This ensures bulk_block outputs are between 0x1001 and 0x2000 bytes for best USB efficiency
      if (usb_buffer_length > (USB_BULK_BUFFER_SIZE_BYTE >> 2)) {

        // construct the queue word to pass to M0
        q_word = (usb_dma_buffer_offset_main_m4 << 16) | usb_buffer_length;

        // update the usb pouinters to point to the other half of the output buffer
        usb_dma_buffer_length_main_m4 = 0;
        usb_dma_buffer_offset_main_m4 = (usb_dma_buffer_offset_main_m4 + (USB_BULK_BUFFER_SIZE_BYTE >> 1)) & USB_BULK_BUFFER_SIZE_MASK;

        // Try to add the command to m0_queue 
        q_head = (m0_queue->q_head & M0_QUEUE_MASK);
        if ( (q_head + 1 - m0_queue->q_tail) & M0_QUEUE_MASK)
        {// Tell the M0_USB core the offset and size of the data to send
          m0_queue->q[q_head] = q_word;    
          m0_queue->q_head = ((q_head + 1) & M0_QUEUE_MASK);
          // Kick the M0_USB core to start the USB transfer
          signal_sev();
        } else {
          // oops - M0 queue is full. Nothing we can do except silently discard
          //GPIO_NOT(PORT_EN_M0_ACTIVE) = PIN_EN_M0_ACTIVE;
        }
      } else {
        usb_dma_buffer_length_main_m4 = usb_buffer_length;
      }
    }
  }
/*
#ifdef USE_PACKING
  usb_bulk_buffer_offset_m4 = &usb_bulk_buffer_offset_uint32_m4;
#endif

  while(true)
  {
    signal_wfe();
  
#ifdef USE_PACKING
*/
  /* Thanks to Pierre HB9FUF for the initial packing proof-of-concept */
  /* The following expands the PoC to use 4 buffers with an improved packing routine above */
/*
  switch(get_usb_buffer_offset_m4())
  {
  case 0:
    if(phase == 0)
    {
      pack((uint16_t*) &usb_bulk_buffer[0x0000], (uint32_t*) &usb_bulk_buffer[0x0000], 0x1000);
    
      set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 1) );    
      signal_sev();
      phase = 1;
    }
    break;
  case 1:
    if(phase == 1)
    {
      pack((uint16_t*) &usb_bulk_buffer[0x2000], (uint32_t*) &usb_bulk_buffer[0x2000], 0x1000);
    
      set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 1) );    
      signal_sev();
      phase = 2;
    }
    break;
  case 2:
    if(phase == 2)
    {
      pack((uint16_t*) &usb_bulk_buffer[0x4000], (uint32_t*) &usb_bulk_buffer[0x4000], 0x1000);
    
      set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 1) );    
      signal_sev();
      phase = 3;
    }
    break;
  case 3:
    if(phase == 3)
    {
      pack((uint16_t*) &usb_bulk_buffer[0x6000], (uint32_t*) &usb_bulk_buffer[0x6000], 0x1000);
    
      set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 1) );    
      signal_sev();
      phase = 0;
    }
    break;
  }
#endif  

  }  
*/
}
