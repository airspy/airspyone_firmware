/*
 * Copyright 2013-2018 Benjamin Vernoux <bvernoux@airspy.com>
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

#define DEFAULT_ADCHS_CHAN (0)

#undef DMA_ISR_DEBUG
//#define DMA_ISR_DEBUG

#define DATA_PACKED_BUFFER_LEN ((ADCHS_DATA_TRANSFER_SIZE_BYTE/2)/16*12)

#define USB_DATA_TRANSFER_SIZE_BYTE (ADCHS_DATA_TRANSFER_SIZE_BYTE)
#define USB_BULK_BUFFER_MASK ((32768) - 1)
#define get_usb_buffer_offset() (usb_bulk_buffer_offset[0])
#define set_usb_buffer_offset(val) (usb_bulk_buffer_offset[0] = val)
/* Manage round robin after increment with USB_BULK_BUFFER_MASK */
#define inc_mask_usb_buffer_offset(buff_offset, inc_value) ((buff_offset+inc_value) & USB_BULK_BUFFER_MASK)

volatile uint32_t usb_bulk_buffer_offset_uint32_m4;
volatile uint32_t *usb_bulk_buffer_offset_m4;
volatile uint32_t last_offset_m4;
#define get_usb_buffer_offset_m4() (usb_bulk_buffer_offset_m4[0])
#define set_usb_buffer_offset_m4(val) (usb_bulk_buffer_offset_m4[0] = val)
#define inc_mask_usb_buffer_offset_m4(buff_offset, inc_value) inc_mask_usb_buffer_offset(buff_offset, inc_value)

#define SLAVE_TXEV_FLAG ((uint32_t *) 0x40043400)
#define SLAVE_TXEV_QUIT() { *SLAVE_TXEV_FLAG = 0x0; }

extern uint32_t cm4_data_share; /* defined in linker script */
extern uint32_t adchs_data; /* defined in linker script */
extern uint32_t cm0_data_share; /* defined in linker script */

volatile int adchs_stopped = 0;
volatile int adchs_started = 0;

volatile airspy_packing_type use_packing = AIRSPY_PACKING_OFF;

volatile uint32_t *usb_bulk_buffer_offset = &cm4_data_share;
volatile uint32_t *usb_bulk_buffer_length = ((&cm4_data_share)+1);
volatile uint32_t *last_offset_m0 = ((&cm4_data_share)+2);

uint8_t* const usb_bulk_buffer = (uint8_t*)USB_BULK_BUFFER_START;

volatile airspy_mcore_t *start_adchs = (airspy_mcore_t *)(&cm0_data_share);
volatile airspy_mcore_t *set_samplerate = (airspy_mcore_t *)((&cm0_data_share)+1);
volatile airspy_mcore_t *set_packing = (airspy_mcore_t *)((&cm0_data_share)+2);

volatile int first_start = 0;

volatile int framecounter = 0;

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

/*
__attribute__ ((always_inline)) static void pack(uint16_t* input, uint32_t* output, uint32_t length)
{
  uint32_t i;
    
  for (i = 0; i < length; i += 8)
  {    
    register uint32_t t2, t5;
    
    t2 = input[i+2];      
        
    output[0] = ((uint32_t)(input[i] << 20)) | ((uint32_t)(input[i+1] << 8)) | (t2 >> 4);    
    t5 = input[i+5];
    output[1] = ((uint32_t)(t2&0xf) << 28)| ((uint32_t)input[i+3] << 16) | (input[i+4] << 4) | (t5 >> 8);
    output[2] = ((uint32_t)(t5 & 0xff) << 24) | ((uint32_t)input[i+6]<<12) | ((uint32_t)input[i+7]);

    output += 3;
  }
}*/

__attribute__ ((always_inline)) static void pack(uint32_t* input, uint32_t* output, uint32_t length)
{
  register uint32_t *a0 asm("r0") = input;
  register uint32_t *a1 asm("r1") = output;
  register uint32_t a2 asm("r2") = length;

  asm volatile("1:\n\t"
         "ldm.w %0!, {r4, r5, r6, r7}\n\t"
         
         "lsr	r8, r4, #16\n\t"
         "ubfx	r3, r5, #4, #12\n\t"
         "orr	r8, r3, r8, lsl #8\n\t"
         "orr	r8, r8, r4, lsl #20\n\t"
         "lsrs	r3, r5, #16\n\t"
         "lsls	r5, r5, #28\n\t"
         "orr	r5, r5, r3, lsl #16\n\t"
         "orr	r5, r5, r6, lsr #24\n\t"
         "uxth	r9, r6\n\t"
         "orr	r9, r5, r9, lsl #4\n\t"
         "lsrs	r6, r6, #16\n\t"
         "uxth	r10, r7\n\t"
         "lsl	r10, r10, #12\n\t"
         "orr	r10, r10, r6, lsl #24\n\t"	
         "orr	r10, r10, r7, lsr #16\n\t"

         "stm.w %1!, {r8, r9, r10}\n\t"

         "subs	%2, %2, #8\n\t"
         "bne 1b\n\t"
        : "+r"(a0), "+r"(a1), "+r"(a2)
        :: "memory", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10");
        
}

static __inline__ void clr_usb_buffer_offset(void)
{  
  if(use_packing == AIRSPY_PACKING_OFF)
  {
    usb_bulk_buffer_offset[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE;
  }
  else /* Other Case Packing is ON */
  {
    usb_bulk_buffer_offset[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE / 2;
    usb_bulk_buffer_offset_m4[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE;
  }
  
  last_offset_m4 = 0;
  *last_offset_m0 = 0;
}

static __inline__ uint32_t get_start_stop_adchs(void)
{
  return(start_adchs->cmd);
}

/* Acknowledge Start/Stop ADCHS by clearing the data */
static __inline__ void ack_start_stop_adchs(void)
{
  start_adchs->raw  = 0;
}

static __inline__ uint8_t get_samplerate(uint8_t *conf_number)
{
  *conf_number = set_samplerate->conf;
  return(set_samplerate->cmd);
}

/* Acknowledge set_samplerate by clearing the data */
static __inline__ void ack_samplerate(void)
{
  set_samplerate->raw = 0;
}

static __inline__ uint8_t get_packing(uint8_t *packing_state)
{
  *packing_state = set_packing->conf;
  return(set_packing->cmd);
}

static __inline__ void ack_packing(void)
{
  set_packing->raw = 0;
}

void set_packing_state(uint8_t state)
{
  if(state == 0)
  {
    use_packing = AIRSPY_PACKING_OFF;
    *usb_bulk_buffer_length = ADCHS_DATA_TRANSFER_SIZE_BYTE;
  } else if (state == 1)
  {
    use_packing = AIRSPY_PACKING_ON;
    *usb_bulk_buffer_length = DATA_PACKED_BUFFER_LEN;
  } else if (state == 2)
  {
    use_packing = AIRSPY_PACKING_TIMESTAMP;
    *usb_bulk_buffer_length = DATA_PACKED_BUFFER_LEN;
  }
}

void adchs_start(uint8_t chan_num)
{
  int i;
  uint32_t *dst;

  /* Disable IRQ globally */
  __asm__("cpsid i");

  if(first_start == 0)
  {
    cpu_clock_pll1_high_speed(&airspy_conf->airspy_m4_init_conf.pll1_hs);
    first_start = 1;
  }

  /* Clear ADCHS Buffer */
  dst = (uint32_t *)ADCHS_DATA_BUFFER;
  for(i=0; i<(ADCHS_DATA_BUFFER_SIZE_BYTE/4); i++)
  {
    dst[i] = 0;
  }
  clr_usb_buffer_offset();
  framecounter = 0;

  ADCHS_init();
  ADCHS_desc_init(chan_num);
  ADCHS_DMA_init((uint32_t)ADCHS_DATA_BUFFER, use_packing);

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

//  cpu_clock_pll1_low_speed(&airspy_conf->airspy_m4_init_conf.pll1_ls);

  led_off();

  /* Enable IRQ globally */
  __asm__("cpsie i");
}

void dma_isr(void) 
{
  uint32_t status;
  #define INTTC0  (1)

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
  if( status & INTTC0 )
  {
    LPC_GPDMA->INTTCCLEAR |= INTTC0; /* Clear Chan0 */

    if(use_packing == AIRSPY_PACKING_TIMESTAMP)
    {
        uint32_t ofs = get_usb_buffer_offset_m4();
        uint32_t* hdr = (uint32_t*)(usb_bulk_buffer+ofs);
        uint32_t ads = LPC_ADCHS->STATUS0;
        LPC_ADCHS->CLR_STAT0 = ads;
        hdr[0]=framecounter|(ads<<16);
        set_usb_buffer_offset_m4( inc_mask_usb_buffer_offset_m4(ofs, USB_DATA_TRANSFER_SIZE_BYTE/2));
        framecounter=(framecounter+1)&4095;
    } else if(use_packing == AIRSPY_PACKING_ON)
    {
        set_usb_buffer_offset_m4( inc_mask_usb_buffer_offset_m4(get_usb_buffer_offset_m4(), USB_DATA_TRANSFER_SIZE_BYTE/2));
    } else // default AIRSPY_PACKING_OFF
    {
        set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), USB_DATA_TRANSFER_SIZE_BYTE) );
        signal_sev();
    }
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
  uint8_t adchs_conf;
  uint8_t adchs_start_stop_cmd;
  uint8_t samplerate_cmd;
  uint8_t packing_cmd;
  uint8_t packing_state;

  SLAVE_TXEV_QUIT();

  samplerate_cmd = get_samplerate(&adchs_conf);
  if(samplerate_cmd == SET_SAMPLERATE_CMD)
  {
    if((adchs_conf & AIRSPY_SAMPLERATE_CONF_ALT) == AIRSPY_SAMPLERATE_CONF_ALT)
    {
      adchs_conf = adchs_conf & (~AIRSPY_SAMPLERATE_CONF_ALT);
      sys_clock_samplerate(&airspy_conf->airspy_m0_m4_alt_conf[adchs_conf].airspy_m4_conf);
    }else
    {
      sys_clock_samplerate(&airspy_conf->airspy_m0_m4_conf[adchs_conf].airspy_m4_conf);
    }
    ack_samplerate();
  }
  
  packing_cmd = get_packing(&packing_state);
  if(packing_cmd == SET_PACKING_CMD)
  {
    set_packing_state(packing_state);
    ack_packing();
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

  /* Halt M0s core (in case it was running) */
  ipc_halt_m0s();

  /* Copy M0s code from M4 embedded addr to final addr M0s */
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
  sys_clock_init();

  nvic_set_priority(NVIC_DMA_IRQ, 255);
  nvic_set_priority(NVIC_M0CORE_IRQ, 1);

  clr_usb_buffer_offset();

  nvic_enable_irq(NVIC_DMA_IRQ);
  nvic_enable_irq(NVIC_M0CORE_IRQ);

  adchs_stop();
  adchs_stopped = 1;
  adchs_started = 0;
  
  use_packing = AIRSPY_PACKING_OFF;
  *usb_bulk_buffer_length = ADCHS_DATA_TRANSFER_SIZE_BYTE;

  ack_start_stop_adchs();
  ack_samplerate();
  ack_packing();

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

  usb_bulk_buffer_offset_m4 = &usb_bulk_buffer_offset_uint32_m4;
  
  while(true)
  {
    signal_wfe();
  
    if(use_packing)
    {
      /* Thanks to Pierre HB9FUF for the initial packing proof-of-concept */    
      uint32_t offset = get_usb_buffer_offset_m4();
      if(offset != last_offset_m4)
      {
        pack((uint32_t*)&usb_bulk_buffer[offset], (uint32_t*)&usb_bulk_buffer[offset], 0x1000);
        set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 0x2000));    
        signal_sev();
        last_offset_m4 = offset;
      }
    }
  }
}
