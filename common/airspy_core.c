/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
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

#include "airspy_core.h"
#include "airspy_conf.h"
#include "airspy_calib.h"
#include "si5351c.h"
#include <libopencm3/lpc43xx/i2c.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/ssp.h>
#include <libopencm3/lpc43xx/rgu.h>

#define ROMFLASH_BASE_ADDR (0x80000000UL)

#define WAIT_CPU_CLOCK_INIT_DELAY (10000)  /* Wait about 150us@200Mhz, 300us@96MHz & 2400us@12Mhz (about 3cycles*20000) */
#define WAIT_R820T_POWER_ON_DELAY (100000) /* Wait about 1500us@200Mhz, 3000us@96MHz & 24000us@12Mhz (about 3cycles*100000) */

typedef struct 
{
  airspy_conf_t airspy_conf;
}airspy_nos_conf_t;

typedef struct 
{
  airspy_conf_t airspy_conf;
}airspy_mini_conf_t;

extern airspy_mini_conf_t __attribute__ ((section(".nocopy_data"))) airspy_mini_conf;
extern airspy_nos_conf_t __attribute__ ((section(".nocopy_data"))) airspy_nos_conf;

airspy_conf_t* airspy_conf = (airspy_conf_t*)AIRSPY_CONF_SRAM_ADDR;

uint8_t si5351c_read[4] = { 0 };

typedef struct 
{
  scu_grp_pin_t group_pin;
  uint32_t scu_conf;
} gpio_conf_t;

#define GPIO_CONF_NB  (161)

// SCU_CONF_EPD_EN_PULLDOWN
/* Disable PullUp on GPIO */
#define SCU_CONF_FUNC0 (SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_FUNCTION0)
#define SCU_CONF_FUNC1 (SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_FUNCTION1)
#define SCU_CONF_FUNC2 (SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_FUNCTION2)
#define SCU_CONF_FUNC4 (SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_FUNCTION4)
#define CLK_IN_FUNC1 (SCU_CLK_IN | SCU_CONF_FUNCTION1)

const gpio_conf_t gpio_conf[GPIO_CONF_NB] =
{
  { P0_0, SCU_CONF_FUNC0 },
  { P0_1, SCU_CONF_FUNC0 },

  { P1_0,  SCU_CONF_FUNC0 },
  { P1_1 , SCU_CONF_FUNC0 }, // SCU_PINMUX_BOOT0
  { P1_2 , SCU_CONF_FUNC0 }, // SCU_PINMUX_BOOT1
  { P1_3 , SCU_CONF_FUNC0 },
  { P1_4 , SCU_CONF_FUNC0 },
  { P1_5 , SCU_CONF_FUNC0 },
  { P1_6 , SCU_CONF_FUNC0 },
  { P1_7 , SCU_CONF_FUNC0 },
  { P1_8 , SCU_CONF_FUNC0 },
  { P1_9 , SCU_CONF_FUNC0 },
  { P1_10, SCU_CONF_FUNC0 },
  { P1_11, SCU_CONF_FUNC0 },
  { P1_12, SCU_CONF_FUNC0 },
  { P1_13, SCU_CONF_FUNC0 },
  { P1_14, SCU_CONF_FUNCTION0 }, // PIN_EN_R820T
  { P1_15, SCU_CONF_FUNC0 },
  { P1_16, SCU_CONF_FUNC0 },
  { P1_17, SCU_CONF_FUNCTION0 }, // PIN_EN_LED1
  { P1_18, SCU_CONF_FUNC0 },
  { P1_19, SCU_CONF_FUNC2 },
  { P1_20, SCU_CONF_FUNC0 },

  { P2_0,  SCU_CONF_FUNC4 },
  { P2_1,  SCU_CONF_FUNC4 },
  { P2_2,  SCU_CONF_FUNC4 },
  { P2_3,  SCU_CONF_FUNC4 },
  { P2_4,  SCU_CONF_FUNC4 },
  { P2_5,  SCU_CONF_FUNC4 },
  { P2_6 , SCU_CONF_FUNC4 },
  { P2_7 , SCU_CONF_FUNC0 },
  { P2_8 , SCU_CONF_FUNC4 }, // SCU_PINMUX_BOOT2
  { P2_9 , SCU_CONF_FUNC0 }, // SCU_PINMUX_BOOT3
  { P2_10, SCU_CONF_FUNC0 },
  { P2_11, SCU_CONF_FUNC0 },
  { P2_12, SCU_CONF_FUNC0 },
  { P2_13, SCU_CONF_FUNCTION0 }, // PIN_EN_BIAST

  { P3_0, SCU_CONF_FUNC0 },
  { P3_1, SCU_CONF_FUNC4 },
  { P3_2, SCU_CONF_FUNC4 },
  // { P3_3, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  // { P3_4, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  // { P3_5, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  // { P3_6, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  // { P3_7, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  // { P3_8, SCU_CONF_FUNC0 }, Reserved by boot for SPIFI
  { P4_0, SCU_CONF_FUNC0 },
  { P4_1, SCU_CONF_FUNC0 },
  { P4_2, SCU_CONF_FUNC0 },
  { P4_3, SCU_CONF_FUNC0 },
  { P4_4, SCU_CONF_FUNC0 },
  { P4_5, SCU_CONF_FUNC0 },
  { P4_6, SCU_CONF_FUNC0 },

  { P4_7 , SCU_CONF_FUNC2 },
  { P4_8 , SCU_CONF_FUNC4 },
  { P4_9 , SCU_CONF_FUNC4 },
  { P4_10, SCU_CONF_FUNC4 },

  { P5_0, SCU_CONF_FUNC0 },
  { P5_1, SCU_CONF_FUNC0 },
  { P5_2, SCU_CONF_FUNC0 },
  { P5_3, SCU_CONF_FUNC0 },
  { P5_4, SCU_CONF_FUNC0 },
  { P5_5, SCU_CONF_FUNC0 },
  { P5_6, SCU_CONF_FUNC0 },
  { P5_7, SCU_CONF_FUNC0 },

  { P6_0 , SCU_CONF_FUNC0 },
  { P6_1 , SCU_CONF_FUNC0 },
  { P6_2 , SCU_CONF_FUNC0 },
  { P6_3 , SCU_CONF_FUNC0 },
  { P6_4 , SCU_CONF_FUNC0 },
  { P6_5 , SCU_CONF_FUNC0 },
  { P6_6 , SCU_CONF_FUNC0 },
  { P6_7 , SCU_CONF_FUNC0 },
  { P6_8 , SCU_CONF_FUNC0 },
  { P6_9 , SCU_CONF_FUNC0 },
  { P6_10, SCU_CONF_FUNC0 },
  { P6_11, SCU_CONF_FUNC0 },
  { P6_12, SCU_CONF_FUNC0 },

  { P7_0, SCU_CONF_FUNC0 },
  { P7_1, SCU_CONF_FUNC0 },
  { P7_2, SCU_CONF_FUNC0 },
  { P7_3, SCU_CONF_FUNC0 },
  { P7_4, SCU_CONF_FUNC0 },
  { P7_5, SCU_CONF_FUNC0 },
  { P7_6, SCU_CONF_FUNC0 },
  { P7_7, SCU_CONF_FUNC0 },

  { P8_0, SCU_CONF_FUNC0 },
  { P8_1, SCU_CONF_FUNC0 },
  { P8_2, SCU_CONF_FUNC0 },
  { P8_3, SCU_CONF_FUNC0 },
  { P8_4, SCU_CONF_FUNC0 },
  { P8_5, SCU_CONF_FUNC0 },
  { P8_6, SCU_CONF_FUNC0 },
  { P8_7, SCU_CONF_FUNC0 },
  { P8_8, SCU_CONF_FUNC0 },

  { PA_0, SCU_CONF_FUNC0 },
  { PA_1, SCU_CONF_FUNC0 },
  { PA_2, SCU_CONF_FUNC0 },
  { PA_3, SCU_CONF_FUNC0 },
  { PA_4, SCU_CONF_FUNC0 },

  { PB_0, SCU_CONF_FUNC0 },
  { PB_1, SCU_CONF_FUNC0 },
  { PB_2, SCU_CONF_FUNC0 },
  { PB_3, SCU_CONF_FUNC0 },
  { PB_4, SCU_CONF_FUNC0 },
  { PB_5, SCU_CONF_FUNC0 },
  { PB_6, SCU_CONF_FUNC0 },

  { PC_0 , SCU_CONF_FUNC0 },
  { PC_1 , SCU_CONF_FUNC4 },
  { PC_2 , SCU_CONF_FUNC4 },
  { PC_3 , SCU_CONF_FUNC4 },
  { PC_4 , SCU_CONF_FUNC0 },
  { PC_5 , SCU_CONF_FUNC0 },
  { PC_6 , SCU_CONF_FUNC0 },
  { PC_7 , SCU_CONF_FUNC0 },
  { PC_8 , SCU_CONF_FUNC0 },
  { PC_9 , SCU_CONF_FUNC0 },
  { PC_10, SCU_CONF_FUNC0 },
  { PC_11, SCU_CONF_FUNC0 },
  { PC_12, SCU_CONF_FUNC0 },
  { PC_13, SCU_CONF_FUNC0 },
  { PC_14, SCU_CONF_FUNC0 },

  { PD_0 , SCU_CONF_FUNC0 },
  { PD_1 , SCU_CONF_FUNC0 },
  { PD_2 , SCU_CONF_FUNC0 },
  { PD_3 , SCU_CONF_FUNC0 },
  { PD_4 , SCU_CONF_FUNC0 },
  { PD_5 , SCU_CONF_FUNC0 },
  { PD_6 , SCU_CONF_FUNC0 },
  { PD_7 , SCU_CONF_FUNC0 },
  { PD_8 , SCU_CONF_FUNC0 },
  { PD_9 , SCU_CONF_FUNC0 },
  { PD_10, SCU_CONF_FUNC0 },
  { PD_11, SCU_CONF_FUNC0 },
  { PD_12, SCU_CONF_FUNC0 },
  { PD_13, SCU_CONF_FUNC0 },
  { PD_14, SCU_CONF_FUNC0 },
  { PD_15, SCU_CONF_FUNC0 },
  { PD_16, SCU_CONF_FUNC0 },

  { PE_0 , SCU_CONF_FUNC0 },
  { PE_1 , SCU_CONF_FUNC0 },
  { PE_2 , SCU_CONF_FUNC4 },
  { PE_3 , SCU_CONF_FUNC0 },
  { PE_4 , SCU_CONF_FUNC0 },
  { PE_5 , SCU_CONF_FUNC0 },
  { PE_6 , SCU_CONF_FUNC0 },
  { PE_7 , SCU_CONF_FUNC0 },
  { PE_8 , SCU_CONF_FUNC0 },
  { PE_9 , SCU_CONF_FUNC0 },
  { PE_10, SCU_CONF_FUNC0 },
  { PE_11, SCU_CONF_FUNC0 },
  { PE_12, SCU_CONF_FUNC0 },
  { PE_13, SCU_CONF_FUNC0 },
  { PE_14, SCU_CONF_FUNC0 },
  { PE_15, SCU_CONF_FUNC0 },

  { PF_0 , SCU_CONF_FUNC1 },
  { PF_1 , SCU_CONF_FUNC0 },
  { PF_2 , SCU_CONF_FUNC0 },
  { PF_3 , SCU_CONF_FUNC0 },
  { PF_4 , CLK_IN_FUNC1 }, // CGU_SRC_GP_CLKIN Clock Input from SI5351C
  { PF_5 , SCU_CONF_FUNC0 },
  { PF_6 , SCU_CONF_FUNC0 },
  { PF_7 , SCU_CONF_FUNC0 },
  { PF_8 , SCU_CONF_FUNC0 },
  { PF_9 , SCU_CONF_FUNC0 },
  { PF_10, SCU_CONF_FUNC0 },
  { PF_11, SCU_CONF_FUNC0 }
};

void delay(uint32_t duration)
{
  uint32_t i;

  for (i = 0; i < duration; i++)
    __asm__("nop");
}

void cpu_reset(void)
{
  RESET_CTRL0 = RESET_CTRL0_CORE_RST;

  while(1);
}

void sys_clock_samplerate(const airspy_sys_samplerate_t* const pt_airspy_sys_samplerate)
{
  uint32_t pll0audio_mdiv;
  uint32_t pll0audio_npdiv;
  uint32_t pll0audio_ctrl_flags;

  pll0audio_mdiv = pt_airspy_sys_samplerate->pll0audio_mdiv;
  pll0audio_npdiv = pt_airspy_sys_samplerate->pll0audio_npdiv;
  pll0audio_ctrl_flags = pt_airspy_sys_samplerate->pll0audio_ctrl_flags;

  /* Power Down ADCHS Clock */
  CGU_BASE_ADCHS_CLK = CGU_BASE_ADCHS_CLK_PD;
  /* Power Down PLL0AUDIO */
  CGU_PLL0AUDIO_CTRL = CGU_PLL0AUDIO_CTRL_PD;
  /* Power Down IDIVB */
  CGU_IDIVB_CTRL = CGU_IDIVB_CTRL_PD;

  if( (pll0audio_mdiv == 0) &&
      (pll0audio_npdiv == 0) )
  {
    if( pt_airspy_sys_samplerate->idivb == 0)
    {
      /* Do not use IDIVB direct connection CGU_SRC_GP_CLKIN to ACHS_CLK */
      /* ADCHS Clock CGU_BASE_ADCHS_CLK => Clock Source CLK_IN */
      CGU_BASE_ADCHS_CLK = CGU_BASE_ADCHS_CLK_AUTOBLOCK
          | CGU_BASE_ADCHS_CLK_CLK_SEL(CGU_SRC_GP_CLKIN);
    }else
    {
      /* ADCHS Clock CGU_BASE_ADCHS_CLK => Clock Source IDIVB */
      /* Use CGU_SRC_GP_CLKIN as clock source for IDIVB */
      CGU_IDIVB_CTRL = CGU_IDIVB_CTRL_IDIV(pt_airspy_sys_samplerate->idivb)
          | CGU_IDIVB_CTRL_AUTOBLOCK
          | CGU_IDIVB_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN);

      /* ADCHS Clock CGU_BASE_ADCHS_CLK => Clock Source CLK_IDIVB */
      CGU_BASE_ADCHS_CLK = CGU_BASE_ADCHS_CLK_AUTOBLOCK
          | CGU_BASE_ADCHS_CLK_CLK_SEL(CGU_SRC_IDIVB);
    }

  }else
  {
    /* ADCHS Clock CGU_BASE_ADCHS_CLK => Clock Source PLL0AUDIO */
    /* Use CGU_SRC_GP_CLKIN as clock source for PLL0AUDIO */
    CGU_PLL0AUDIO_CTRL = CGU_PLL0AUDIO_CTRL_PD
        | CGU_PLL0AUDIO_CTRL_AUTOBLOCK
        | CGU_PLL0AUDIO_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN);
    while (CGU_PLL0AUDIO_STAT & CGU_PLL0AUDIO_STAT_LOCK);

    /* configure PLL0AUDIO to produce xxMHz */
    /* PLL Register settings (SEL_EXT=1):
       Mdec=31=PLL0_MDIV[16:0] => CGU_PLL0AUDIO_MDIV
       Ndec=0=PLL0_NPDIV[21:12], Pdec=21=PLL0_NPDIV[6:0] => CGU_PLL0AUDIO_NP_DIV
   */
    CGU_PLL0AUDIO_MDIV = pll0audio_mdiv;
    CGU_PLL0AUDIO_NP_DIV = pll0audio_npdiv;
    CGU_PLL0AUDIO_CTRL |= (CGU_PLL0AUDIO_CTRL_PD
        | pll0audio_ctrl_flags
        | CGU_PLL0AUDIO_CTRL_SEL_EXT
        | CGU_PLL0AUDIO_CTRL_CLKEN);

    /* power on PLL0AUDIO and wait until stable */
    CGU_PLL0AUDIO_CTRL &= ~CGU_PLL0AUDIO_CTRL_PD;
    while (!(CGU_PLL0AUDIO_STAT & CGU_PLL0AUDIO_STAT_LOCK));

    /* use PLL0AUDIO as clock source for ADCHS */
    CGU_BASE_ADCHS_CLK = CGU_BASE_ADCHS_CLK_AUTOBLOCK
        | CGU_BASE_ADCHS_CLK_CLK_SEL(CGU_SRC_PLL0AUDIO);
  }
}

/* Return corrected xtal_freq */
uint32_t sys_calib_r820t(uint32_t xtal_freq, int32_t correction_ppb)
{
  const int invppb = 1000000000;
  xtal_freq += ((int64_t)xtal_freq * (int64_t)correction_ppb + invppb / 2) / invppb;
  return xtal_freq;
}

/*
Configure PLL1 to min speed (48MHz) => see cpu_clock_pll1_low_speed() .
Configure PLL0USB @480MHz for USB0.
Note: PLL1 clock is used by M4/M0 core, Peripheral, APB1, APB3.
*/
void sys_clock_init(void) 
{
  const airspy_sys_clock_t* pt_airspy_sys_conf;
  unsigned char* src;
  unsigned char* dest;
  uint32_t addr;
  uint16_t sizeof_struct_u16;
  uint16_t nb_struct_u16;
  airspy_calib_t airspy_calib = { 0 };
  airspy_calib_t* airspy_calib_flash;

  /* After boot the CPU runs at 96 MHz */
  /* cpu runs from: IRC (12MHz) >> PLL M = 24, FCCO @ 288 MHz direct mode >> IDIVC = 4 >> 96 MHz */

  /*
   * 12MHz clock is entering LPC XTAL1/OSC input now.
   */
  /* set xtal oscillator to low frequency mode */
  CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_HF;

  /* power on the oscillator and wait until stable */
  CGU_XTAL_OSC_CTRL &= ~CGU_XTAL_OSC_CTRL_ENABLE;

  /* Wait at least 250us after Crystal Power ON (Wait for crystal to stabilize) 
     defined in User Manual 10503.pdf Rev1.8 See Fig 30. BASE_M4_CLK ramp-up procedure 
  */
  delay(WAIT_CPU_CLOCK_INIT_DELAY);

  /* Use CGU_SRC_XTAL as clock source for BASE_M4_CLK (CPU) */
  CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_XTAL) | CGU_BASE_M4_CLK_AUTOBLOCK);

  /* Use CGU_SRC_XTAL as clock source for Peripheral */
  CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK
      | CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_XTAL);

  /* Use CGU_SRC_XTAL as clock source for APB1 */
  CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK
      | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_XTAL);

  /* Use CGU_SRC_XTAL as clock source for APB3 */
  CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK
      | CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_XTAL);

  /* ********************** */
  /*  SI5351c configuration */
  /* ********************** */
  /*
   * xxMHz clock is entering LPC GP_CLKIN (from SI5351C) input now.
   * AirSpy clocks:
   *   CLK0 -> R820T xxMHz (XTAL_I)
   *   CLK1 -> LPC4370 RTC 32KHz
   *   CLK2 -> SGPIO Clock (not used for future)
   *   CLK3 -> NC
   *   CLK4 -> NC
   *   CLK5 -> NC
   *   CLK6 -> NC
   *   CLK7 -> LPC4370 Main Clock xxMHz.
   */
  /* Configure I2C0 (for SI5351C) to about 375kHz (12MHz/(2*16)=0.375MHz) when we switch over to APB1 clock = 12MHz */
  i2c0_init(16);

  if (si5351c_disable_oeb_pin_control() == true)
  {
    /* SI5351C detected continue init using AirSpy NOS configuration */
    addr = (uint32_t)&airspy_nos_conf;
    addr = (addr | ROMFLASH_BASE_ADDR); /* Fix with Addr from ROMFLASH */
    src = (unsigned char *)addr;
    /* Copy the configuration from Flash to SRAM */
    for (dest = (unsigned char *)airspy_conf; (uint32_t)dest < (((uint32_t)airspy_conf) + AIRSPY_CONF_MAX_DATA_SIZE); )
    {
      *dest++ = *src++;
    }
  }else
  {
    /* SI5351C not detected continue init using AirSpy MINI configuration */
    addr = (uint32_t)&airspy_mini_conf;
    addr = (addr | ROMFLASH_BASE_ADDR); /* Fix with Addr from ROMFLASH */
    src = (unsigned char *)addr;
    /* Copy the configuration from Flash to SRAM */
    for (dest = (unsigned char *)airspy_conf; (uint32_t)dest < (((uint32_t)airspy_conf) + AIRSPY_CONF_MAX_DATA_SIZE); )
    {
      *dest++ = *src++;
    }
  }

  /* Compute & update 1st, 2nd & 3rd Expansion Conf Point Addr */
  addr = ((uint32_t)(&airspy_conf->si5351c_config)) + (sizeof(void *));
  /* 1st Expansion Conf Point M0/M4 Addr */
  airspy_conf->airspy_m0_m4_conf = (airspy_m0_m4_conf_t*)addr;
  sizeof_struct_u16 = airspy_conf->sizeof_airspy_m0_m4_conf_t;
  nb_struct_u16 = airspy_conf->nb_airspy_m0_m4_conf_t;

  addr += (sizeof_struct_u16 * nb_struct_u16);
  /* 2nd Expansion Conf Point M0/M4 ALT Addr */ 
  airspy_conf->airspy_m0_m4_alt_conf = (airspy_m0_m4_conf_t*)addr;
  sizeof_struct_u16 = airspy_conf->sizeof_airspy_m0_m4_alt_conf_t;
  nb_struct_u16 = airspy_conf->nb_airspy_m0_m4_alt_conf_t;

  addr += (sizeof_struct_u16 * nb_struct_u16);
  /* 3rd Expansion Conf Point SI5351C Addr */ 
  airspy_conf->si5351c_config = (si5351c_conf_t*)addr;

  /* Set default r820t_conf_rw.if_freq to airspy_m0_m4_conf[0] => r820t_if_freq  */
  airspy_conf->r820t_conf_rw.if_freq = airspy_conf->airspy_m0_m4_conf[0].airspy_m0_conf.r820t_if_freq;

  /* Load calibration data */
  addr = (ROMFLASH_BASE_ADDR + AIRSPY_FLASH_CALIB_OFFSET); /* Addr from Flash Configuration 0 (Calibration Data) */
  airspy_calib_flash = (airspy_calib_t*)(addr);
  airspy_calib.header = airspy_calib_flash->header;
  airspy_calib.timestamp = airspy_calib_flash->timestamp;
  airspy_calib.correction_ppb = airspy_calib_flash->correction_ppb;

  if((airspy_conf->conf_hw.hardware_type & HW_FEATURE_SI5351C) == HW_FEATURE_SI5351C)
  {
    /* Programming the Si5351 via I2C
       http://community.silabs.com/t5/Silicon-Labs-Knowledge-Base/Programming-the-Si5351-via-I2C/ta-p/112251
    */
    si5351c_disable_all_outputs();
    si5351c_init_fanout();
    si5351c_power_down_all_clocks();
    si5351c_init_xtal();
    si5351c_read[0] = si5351c_read_single(0);

    /* Configure and enable SI5351C clocks */
    si5351c_read[1] = (si5351c_read_single(0) & SI5351C_REG0_CLKIN_LOS);

    /* CLKIN Loss Of Signal (LOS) ? */
    if(si5351c_read[1] == SI5351C_REG0_CLKIN_LOS)
    {
      /* Apply SI5351C configuration */
      si5351c_airspy_config(&airspy_conf->si5351c_config[AIRSPY_SI5351C_CONFIG_XTAL]);

      /* Check calibration is valid / enabled */
      if(airspy_calib.header == AIRSPY_FLASH_CALIB_HEADER)
      {
        airspy_conf->r820t_conf_rw.xtal_freq = sys_calib_r820t(airspy_conf->r820t_conf_rw.xtal_freq, airspy_calib.correction_ppb);
      }
    }else
    {
        si5351c_airspy_config(&airspy_conf->si5351c_config[AIRSPY_SI5351C_CONFIG_CLKIN]);
    }

    si5351c_read[2] = si5351c_read_single(0);

    si5351c_init_pll_soft_reset();
    si5351c_enable_clock_outputs();

    /* Wait at least 300us after SI5351C Clock Enable */
    delay(WAIT_CPU_CLOCK_INIT_DELAY);
    si5351c_read[3] = si5351c_read_single(0);
  }else
  {
    /* Check calibration is valid / enabled */
    if(airspy_calib.header == AIRSPY_FLASH_CALIB_HEADER)
    {
      airspy_conf->r820t_conf_rw.xtal_freq = sys_calib_r820t(airspy_conf->r820t_conf_rw.xtal_freq, airspy_calib.correction_ppb);
    }
  }

  pt_airspy_sys_conf = &airspy_conf->airspy_m4_init_conf;

  /* ********************************************************************* */
  /*  M4/M0 core, Peripheral, APB1, APB3 Configuration (PLL1 clock source) */
  /* ********************************************************************* */
  /* Configure PLL1 with CGU_SRC_GP_CLKIN as source clock */
  cpu_clock_pll1_low_speed(&pt_airspy_sys_conf->pll1_ls);

  /* Configure I2C0 (for SI5351C) to 400kHz when we switch over to APB1 clock = PLL1 */
  i2c0_init(airspy_conf->i2c_conf.i2c0_pll1_ls_hs_conf_val);
  /* Configure I2C1 (for R820T) to 400kHz when we switch over to APB3 clock = PLL1 */
  i2c1_init(airspy_conf->i2c_conf.i2c1_pll1_ls_conf_val);

  /* ************************************************** */
  /* Connect PLL1 to M4/M0 core, Peripheral, APB1, APB3 */
  /* ************************************************** */
  /* Use PLL1 as clock source for BASE_M4_CLK (CPU) */
  CGU_BASE_M4_CLK = (CGU_BASE_M4_CLK_CLK_SEL(CGU_SRC_PLL1) | CGU_BASE_M4_CLK_AUTOBLOCK);

  /* Switch peripheral clock over to use PLL1 */
  CGU_BASE_PERIPH_CLK = CGU_BASE_PERIPH_CLK_AUTOBLOCK
      | CGU_BASE_PERIPH_CLK_CLK_SEL(CGU_SRC_PLL1);

  /* Switch APB1 clock over to use PLL1 */
  CGU_BASE_APB1_CLK = CGU_BASE_APB1_CLK_AUTOBLOCK
      | CGU_BASE_APB1_CLK_CLK_SEL(CGU_SRC_PLL1);

  /* Switch APB3 clock over to use PLL1 */
  CGU_BASE_APB3_CLK = CGU_BASE_APB3_CLK_AUTOBLOCK
      | CGU_BASE_APB3_CLK_CLK_SEL(CGU_SRC_PLL1);

  /* **************************************************** */
  /* PLL0USB & USB0 Configuration (GP_CLKIN clock source) */
  /* **************************************************** */
  /* Use CGU_SRC_GP_CLKIN as clock source for PLL0USB */
  CGU_PLL0USB_CTRL = CGU_PLL0USB_CTRL_PD
      | CGU_PLL0USB_CTRL_AUTOBLOCK
      | CGU_PLL0USB_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN);
  while (CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK);

  /* configure PLL0USB to produce 480 MHz clock from CGU_SRC_GP_CLKIN */
  CGU_PLL0USB_MDIV = pt_airspy_sys_conf->pll0_usb_mdiv;
  CGU_PLL0USB_NP_DIV = pt_airspy_sys_conf->pll0_usb_npdiv;
  CGU_PLL0USB_CTRL |= (CGU_PLL0USB_CTRL_PD
      | pt_airspy_sys_conf->pll0usb_ctrl_flags
      | CGU_PLL0USB_CTRL_CLKEN);

  /* Power on PLL0USB and wait until stable */
  CGU_PLL0USB_CTRL &= ~CGU_PLL0USB_CTRL_PD;
  while (!(CGU_PLL0USB_STAT & CGU_PLL0USB_STAT_LOCK));

  /* Use PLL0USB as clock source for USB0 */
  CGU_BASE_USB0_CLK = CGU_BASE_USB0_CLK_AUTOBLOCK
      | CGU_BASE_USB0_CLK_CLK_SEL(CGU_SRC_PLL0USB);

  /* ****************************************** */
  /* Disable/PowerDown unused clock/peripherals */
  /* ****************************************** */
  CREG_CREG6 |= (1<<17); // PowerDown RNG

  /* Disable XTAL because GP_CLKIN is used from SI5351C instead */
  /* Switch off the oscillator */
  CGU_XTAL_OSC_CTRL = CGU_XTAL_OSC_CTRL_ENABLE;

  CGU_BASE_SAFE_CLK = CGU_BASE_USB1_CLK_PD;
  // CGU_BASE_USB0_CLK is used for USB0 HS
  // CGU_BASE_M0_CLK is used
  /* Switch off USB1 clock */
  CGU_BASE_USB1_CLK = CGU_BASE_USB1_CLK_PD;
  // CGU_BASE_M4_CLK is used
  CGU_BASE_SPIFI_CLK = CGU_BASE_SPIFI_CLK_PD;
  /* Switch off SPI clock */
  CGU_BASE_SPI_CLK = CGU_BASE_SPI_CLK_PD;
  /* Switch off PHY RX & TX clock */
  CGU_BASE_PHY_RX_CLK = CGU_BASE_PHY_RX_CLK_PD;
  CGU_BASE_PHY_TX_CLK = CGU_BASE_PHY_TX_CLK_PD;
  // CGU_BASE_APB1_CLK is used for I2C0
  // CGU_BASE_APB3_CLK is used for I2C1
  /* Switch off LCD clock */
  CGU_BASE_LCD_CLK = CGU_BASE_LCD_CLK_PD;
  // CGU_BASE_ADCHS_CLK is used
  /* Switch off SDIO clock */
  CGU_BASE_SDIO_CLK = CGU_BASE_SDIO_CLK_PD;
  CGU_BASE_SSP0_CLK = CGU_BASE_SSP0_CLK_PD;
  /* Switch off SSP1 clock */
  CGU_BASE_SSP1_CLK = CGU_BASE_SSP1_CLK_PD;
  /* Switch off UART0 to 3 clock */
  CGU_BASE_UART0_CLK = CGU_BASE_UART0_CLK_PD;
  CGU_BASE_UART1_CLK = CGU_BASE_UART1_CLK_PD;
  CGU_BASE_UART2_CLK = CGU_BASE_UART2_CLK_PD;
  CGU_BASE_UART3_CLK = CGU_BASE_UART3_CLK_PD;
  /*  Switch off OUT clocks */
  CGU_BASE_OUT_CLK = CGU_BASE_OUT_CLK_PD;
  /* Reserved/Undocumented clocks power down */
  CGU_OUTCLK_21_CTRL = 1;
  CGU_OUTCLK_22_CTRL = 1;
  CGU_OUTCLK_23_CTRL = 1;
  CGU_OUTCLK_24_CTRL = 1;
  /* Switch off AUDIO clock */
  CGU_BASE_APLL_CLK = CGU_BASE_APLL_CLK_PD;
  CGU_BASE_CGU_OUT0_CLK = CGU_BASE_CGU_OUT0_CLK_PD;
  CGU_BASE_CGU_OUT1_CLK = CGU_BASE_CGU_OUT1_CLK_PD;
  /* Switch off IDIV C,D,E disabled */
  CGU_IDIVC_CTRL = CGU_IDIVC_CTRL_PD;
  CGU_IDIVD_CTRL = CGU_IDIVD_CTRL_PD;
  CGU_IDIVE_CTRL = CGU_IDIVE_CTRL_PD;
/*
  // Power down M4 branches, but not BUS, GPIO, CREG and M0 & M4 CORE clock
*/
  //CCU1_CLK_M4_BUS_CFG &= ~(1);
  CCU1_CLK_M4_SPIFI_CFG &= ~(1);
  //CCU1_CLK_M4_GPIO_CFG &= ~(1);
  CCU1_CLK_M4_LCD_CFG &= ~(1);
  CCU1_CLK_M4_ETHERNET_CFG &= ~(1);
  //CCU1_CLK_M4_USB0_CFG &= ~(1);
  CCU1_CLK_M4_EMC_CFG &= ~(1);
  CCU1_CLK_M4_SDIO_CFG &= ~(1);
  //CCU1_CLK_M4_DMA_CFG &= ~(1);
  //CCU1_CLK_M4_M4CORE_CFG &= ~(1);
  CCU1_CLK_M4_SCT_CFG &= ~(1);
  CCU1_CLK_M4_USB1_CFG &= ~(1);
  CCU1_CLK_M4_EMCDIV_CFG &= ~(1);
  //CCU1_CLK_M4_M0APP_CFG &= ~(1);
  //CCU1_CLK_M4_VADC_CFG &= ~(1);
  CCU1_CLK_M4_WWDT_CFG &= ~(1);
  CCU1_CLK_M4_USART0_CFG &= ~(1);
  CCU1_CLK_M4_UART1_CFG &= ~(1);
  CCU1_CLK_M4_SSP0_CFG &= ~(1);
  CCU1_CLK_M4_SSP1_CFG &= ~(1);
  CCU1_CLK_M4_TIMER0_CFG &= ~(1);
  CCU1_CLK_M4_TIMER1_CFG &= ~(1);
  //CCU1_CLK_M4_SCU_CFG &= ~(1);
  //CCU1_CLK_M4_CREG_CFG &= ~(1);
  CCU1_CLK_M4_RITIMER_CFG &= ~(1);
  CCU1_CLK_M4_USART2_CFG &= ~(1);
  CCU1_CLK_M4_USART3_CFG &= ~(1);
  CCU1_CLK_M4_TIMER2_CFG &= ~(1);
  CCU1_CLK_M4_TIMER3_CFG &= ~(1);

  CCU1_CLK_M4_QEI_CFG &= ~(1);

  CCU1_CLK_PERIPH_SGPIO_CFG &= ~(1);

  /* ******************************************** */
  /*  ADCHS Configuration (GP_CLKIN clock source) */
  /* ******************************************** */
  sys_clock_samplerate(&airspy_conf->airspy_m0_m4_conf[0].airspy_m4_conf);
}

/*
 Configure PLL1 to low speed with CGU_SRC_GP_CLKIN.
 Connect PLL1 to M4/M0 core, Peripheral, APB1, APB3.
 Init I2C0 & I2C1.
This function is mainly used to lower power consumption.
*/
void cpu_clock_pll1_low_speed(const airspy_pll1_ls_t* const pt_airspy_pll1_ls_conf)
{
  uint32_t pll_reg;

  uint32_t pll1_psel;
  uint32_t pll1_nsel;
  uint32_t pll1_msel;

  pll1_psel = pt_airspy_pll1_ls_conf->pll1_ls_psel;
  pll1_nsel = pt_airspy_pll1_ls_conf->pll1_ls_nsel;
  pll1_msel = pt_airspy_pll1_ls_conf->pll1_ls_msel;

  /* Configure PLL1 Clock */
  /* Integer mode:
    FCLKOUT = M*(FCLKIN/N)
    FCCO = 2*P*FCLKOUT = 2*P*M*(FCLKIN/N)
  */
  pll_reg = CGU_PLL1_CTRL;
  /* Clear PLL1 bits */
  pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD | CGU_PLL1_CTRL_FBSEL |  /* CLK SEL, PowerDown , FBSEL */
          CGU_PLL1_CTRL_BYPASS | /* BYPASS */
          CGU_PLL1_CTRL_DIRECT | /* DIRECT */
          CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
  /* Set PLL1 */
  pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN)
        | CGU_PLL1_CTRL_PSEL(pll1_psel)
        | CGU_PLL1_CTRL_NSEL(pll1_nsel)
        | CGU_PLL1_CTRL_MSEL(pll1_msel)
        | CGU_PLL1_CTRL_FBSEL
        | CGU_PLL1_CTRL_DIRECT;
  CGU_PLL1_CTRL = pll_reg;
  /* wait until stable */
  while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK));

}

/*
 Configure PLL1 to high speed with CGU_SRC_GP_CLKIN.
(PLL1 High Speed compliant with UM10503 Rev. 1.8 — 28 January 2014 Fig 30. BASE_M4_CLK ramp-up procedure)
 Connect PLL1 to M4/M0 core, Peripheral, APB1, APB3.
*/
void cpu_clock_pll1_high_speed(const airspy_pll1_hs_t* const pt_airspy_pll1_hs_conf)
{
  uint32_t pll_reg;

  uint32_t pll1_psel;
  uint32_t pll1_nsel;
  uint32_t pll1_msel;

  pll1_psel = pt_airspy_pll1_hs_conf->pll1_hs_psel;
  pll1_nsel = pt_airspy_pll1_hs_conf->pll1_hs_nsel;
  pll1_msel = pt_airspy_pll1_hs_conf->pll1_hs_msel;

  /* Configure PLL1 to Intermediate Clock final freq / 2 (because DIRECT=1) */
  /* Integer mode:
    FCLKOUT = M*(FCLKIN/N)
    FCCO = 2*P*FCLKOUT = 2*P*M*(FCLKIN/N)
  */
  pll_reg = CGU_PLL1_CTRL;
  /* Clear PLL1 bits */
  pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD | CGU_PLL1_CTRL_FBSEL |  /* CLK SEL, PowerDown , FBSEL */
          CGU_PLL1_CTRL_BYPASS | /* BYPASS */
          CGU_PLL1_CTRL_DIRECT | /* DIRECT */
          CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
  
  /* Set PLL1 to High Speed/2 */
  pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN)
        | CGU_PLL1_CTRL_PSEL(pll1_psel)
        | CGU_PLL1_CTRL_NSEL(pll1_nsel)
        | CGU_PLL1_CTRL_MSEL(pll1_msel)
        | CGU_PLL1_CTRL_FBSEL;
  CGU_PLL1_CTRL = pll_reg;

  /* wait until stable */
  while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK));

  /* 
  Wait before to switch to max speed (at least 50us) 
  See UM10503 Rev1.8 Fig 30. BASE_M4_CLK ramp-up procedure Page 150 / 1420
  */
  delay(WAIT_CPU_CLOCK_INIT_DELAY);

  /* Configure PLL1 High Speed (DIRECT=1) */
  /* Direct mode: FCLKOUT = FCCO = M*(FCLKIN/N) */
  pll_reg = CGU_PLL1_CTRL;
  /* Clear PLL1 bits */
  pll_reg &= ~( CGU_PLL1_CTRL_CLK_SEL_MASK | CGU_PLL1_CTRL_PD | CGU_PLL1_CTRL_FBSEL |  /* CLK SEL, PowerDown , FBSEL */
          CGU_PLL1_CTRL_BYPASS | /* BYPASS */
          CGU_PLL1_CTRL_DIRECT | /* DIRECT */
          CGU_PLL1_CTRL_PSEL_MASK | CGU_PLL1_CTRL_MSEL_MASK | CGU_PLL1_CTRL_NSEL_MASK ); /* PSEL, MSEL, NSEL- divider ratios */
  /* Set PLL1 to HighSpeed */
  pll_reg |= CGU_PLL1_CTRL_CLK_SEL(CGU_SRC_GP_CLKIN)
        | CGU_PLL1_CTRL_PSEL(pll1_psel)
        | CGU_PLL1_CTRL_NSEL(pll1_nsel)
        | CGU_PLL1_CTRL_MSEL(pll1_msel)
        | CGU_PLL1_CTRL_FBSEL
        | CGU_PLL1_CTRL_DIRECT;
  CGU_PLL1_CTRL = pll_reg;
  /* wait until stable */
  while (!(CGU_PLL1_STAT & CGU_PLL1_STAT_LOCK));
}

void led_on(void)
{
  gpio_set(PORT_EN_LED1, PIN_EN_LED1);
}

void led_off(void)
{
  gpio_clear(PORT_EN_LED1, PIN_EN_LED1);
}

void pin_setup(void)
{
  int i;

  /* Configure all GPIO as Input (safe state) */
  GPIO0_DIR = 0;
  GPIO1_DIR = 0;
  GPIO2_DIR = 0;
  GPIO3_DIR = 0;
  GPIO4_DIR = 0;
  GPIO5_DIR = 0;
  GPIO6_DIR = 0;
  GPIO7_DIR = 0;

  /* Pin configuration for all pins */
  for(i = 0; i < GPIO_CONF_NB; i++)
  {
    scu_pinmux(gpio_conf[i].group_pin, gpio_conf[i].scu_conf);
  }

  /* GPIO1[7] on P1_14 as output. */
  GPIO1_DIR |= PIN_EN_R820T;
  enable_r820t_power();

  /* GPIO1[13] on P2_13 as output. */
  GPIO1_DIR |= PIN_EN_BIAST;
  disable_biast_power();

  /* GPIO0[12] on P1_17 as output. */
  GPIO0_DIR |= PIN_EN_LED1;

  led_off();
}

void enable_r820t_power(void)
{
  gpio_set(PORT_EN_R820T, PIN_EN_R820T);
  /* Wait after PowerOn (stabilization of LDO & Internal Init of R820T) */
  delay(WAIT_R820T_POWER_ON_DELAY);
}

void enable_biast_power(void)
{
  gpio_set(PORT_EN_BIAST, PIN_EN_BIAST);
}

void disable_biast_power(void)
{
  gpio_clear(PORT_EN_BIAST, PIN_EN_BIAST);
}

