/*
 * Copyright 2016 Benjamin Vernoux <bvernoux@airspy.com>
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
#include "airspy_conf.h"

#define AIRSPY_CONF_M0_M4_NB (2)
#define AIRSPY_CONF_M0_M4_ALT_NB (2)
#define AIRSPY_CONF_SI5351C_NB (2)
#define NULL_ADDR (0)

typedef struct
{
  airspy_conf_t airspy_conf;

  /* 1st Expansion Conf Point M0/M4 Data (mandatory) */
  airspy_m0_m4_conf_t airspy_m0_m4_conf[AIRSPY_CONF_M0_M4_NB];

  /* 2nd Expansion Conf Point M0/M4 ALT Data (optional) */ 
  airspy_m0_m4_conf_t airspy_m0_m4_alt_conf[AIRSPY_CONF_M0_M4_ALT_NB];

  /* 3rd Expansion Conf Point SI5351C Data (depending on conf_hw.hardware_type) */
  si5351c_conf_t si5351c_config[AIRSPY_CONF_SI5351C_NB];

} airspy_nos_conf_t;

airspy_nos_conf_t __attribute__ ((section(".nocopy_data"))) airspy_nos_conf =
{
  /*  airspy_conf_t airspy_conf = */
  {
    /* conf_struct_version_t conf_struct_version = */
    {
      1, 0, 0
    },

    /* version_conf_t conf_hw = */
    {
      HW_TYPE_AIRSPY_NOS,
      "AirSpy NOS"
    },

    /* i2c_conf_t i2c_conf = */
    {
        /* Configure I2C0 & IC21 to 400kHz (140MHz/(2*175)=0.4MHz) */
        /* Also i2c0_pll1_ls_hs_conf_val => I2C val (For 400KHz) = (5*FreqPLL1MHz)/4 */
        175, /* uint16_t i2c0_pll1_ls_hs_conf_val; Si5351C */
        175, /* uint16_t i2c1_pll1_hs_conf_val; R820T2 */
        /* Configure I2C1 to 400kHz (40MHz/(2*50)=0.4MHz) */
        50, /* uint16_t i2c1_pll1_ls_conf_val; R820T2 */
        0, /* padding */
        0 /* spare */
    }, /* End i2c_conf_t i2c_conf */

    /*
    Clock configuration with GP_CLKIN=20MHz
    Configuration of clocks at init (sys_clock_init())
    */
    /* const airspy_sys_clock_t airspy_m4_init_conf = */
    {
      /* Configure PLL0USB to produce 480MHz clock from GP_CLKIN */
      0x040E03FF, // uint32_t pll0_usb_mdiv; SELR SELI SELP MDEC
      0x00000000, // uint32_t pll0_usb_npdiv; PDEC NDEC
      (PLL0USB_CTRL_FLAG_DIRECT_I | PLL0USB_CTRL_FLAG_DIRECT_O), // uint32_t pll0usb_ctrl_flags; DirectI=PLL0USB_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0USB_CTRL_FLAG_DIRECT_O */
      /* PLL1 clock from GP_CLKIN */
      /* PLL1 High Speed Mode Set PLL1 to 20MHz * (6+1) = 140MHz */
      {
        0, // uint32_t pll1_hs_psel;
        0, // uint32_t pll1_hs_nsel;
        6, // uint32_t pll1_hs_msel;
      },
      /* PLL1 Low Speed Mode => Set PLL1 to 20MHz * (1+1) = 40MHz */
      {
        0, // uint32_t pll1_ls_psel;
        0, // uint32_t pll1_ls_nsel;
        1, // uint32_t pll1_ls_msel;
      }
    }, /* End airspy_sys_clock_t airspy_m4_init_conf */

    /* Configuration for R820T2 initial values */
    /* Those initial values start from REG_SHADOW_START */
    /* Shall be not set as const as the structure is used as IN & OUT */
    /* r820t_priv_t r820t_conf_rw = */
    {
      20000000, // xtal_freq => 20MHz
      5000000, // Freq 20MHz => 10Mhz IQ => IF Freq = 5MHz (r820t_if_freq)
      100000000, /* Default Freq 100Mhz */
      {
        /* 05 */ 0x90, // LNA manual gain mode, init to 0
        /* 06 */ 0x80,
        /* 07 */ 0x60,
        /* 08 */ 0x80, // Image Gain Adjustment
        /* 09 */ 0x40, // Image Phase Adjustment
        /* 0A */ 0xA8, // Channel filter [0..3]: 0 = widest, f = narrowest - Optimal. Don't touch!
        /* 0B */ 0x0F, // High pass filter - Optimal. Don't touch!
        /* 0C */ 0x40, // VGA control by code, init at 0
        /* 0D */ 0x63, // LNA AGC settings: [0..3]: Lower threshold; [4..7]: High threshold
        /* 0E */ 0x75,
        /* 0F */ 0xF8, // Filter Widest, LDO_5V OFF, clk out OFF,
        /* 10 */ 0x7C,
        /* 11 */ 0x83,
        /* 12 */ 0x80,
        /* 13 */ 0x00,
        /* 14 */ 0x0F,
        /* 15 */ 0x00,
        /* 16 */ 0xC0,
        /* 17 */ 0x30,
        /* 18 */ 0x48,
        /* 19 */ 0xCC,
        /* 1A */ 0x60,
        /* 1B */ 0x00,
        /* 1C */ 0x54,
        /* 1D */ 0xAE,
        /* 1E */ 0x0A,
        /* 1F */ 0xC0
      },
      0 /* uint16_t padding */
    }, /* End r820t_priv_t r820t_conf_rw */

    /* 1st Expansion Conf Point M0/M4 */
    (uint16_t)(sizeof(airspy_m0_m4_conf_t)), /* uint16_t sizeof_airspy_m0_m4_conf_t */
    AIRSPY_CONF_M0_M4_NB, /* uint16_t nb_airspy_m0_m4_conf_t */
    NULL_ADDR, /* airspy_m0_m4_conf_t* airspy_m0_m4_conf */

    /* 2nd Expansion Conf Point ALT M0/M4 */ 
    (uint16_t)(sizeof(airspy_m0_m4_conf_t)), /* uint16_t sizeof_airspy_m0_m4_alt_conf_t */
    AIRSPY_CONF_M0_M4_ALT_NB, /* uint16_t nb_airspy_m0_m4_alt_conf_t */
    NULL_ADDR, /* airspy_m0_m4_conf_t* airspy_m0_m4_alt_conf */

    /* 3rd Expansion Conf Point SI5351C */ 
    (uint16_t)(sizeof(si5351c_conf_t)), /* uint16_t sizeof_si5351c_conf_t */
    AIRSPY_CONF_SI5351C_NB, /* uint16_t nb_si5351c_conf_t */
    NULL_ADDR /* si5351c_conf_t* si5351c_config */
  },

  /* 1st Expansion Conf Point M0/M4 Data */
  /*
  SampleRate configuration with GP_CLKIN=20MHz
  Configuration of PLL0AUDIO shall not exceed 80MHz for ADCHS
  For PLL0AUDIO sys_clock_samplerate() set SEL_EXT to 1 => MDEC enabled. Fractional divider not used.
  For PLL0AUDIO see UM10503 Rev1.8 "Fig 34. PLL0 with fractional divider" Page 184 / 1420 for more details.
  Configuration of IDIVB => Integer divider B divider values (1/(IDIV + 1))
  */
  /* const airspy_m0_m4_conf_t airspy_m0_m4_conf[AIRSPY_CONF_M0_M4_NB] = */
  {
    /* Conf 0 => 10 MSPS */
    {
      /*
        CGU_SRC_GP_CLKIN=20MHz from SI5351C CLK7 LPC4370 GP_CLKIN
        airspy_sys_samplerate_t airspy_m4_conf
      */
      {
        /* PLL0AUDIO */
        0x00000000, // uint32_t pll0audio_mdiv;
        0x00000000, // uint32_t pll0audio_npdiv;
        0x00000000, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
        /* IDIVB (from GP_CLKIN) */
        0, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
        { 0, 0, 0 } /* uint8_t padding[3] */
      },
      /* airspy_m0_conf_t airspy_m0_conf */
      {
        5000000, // Freq 20MHz => 10MHz IQ => IF Freq = 5MHz (r820t_if_freq)
        59,  // uint8_t r820t_bw;
        0,// uint8_t padding0;
        0 // uint16_t padding1;
      }
    },
    /* Conf 1 => 2.5 MSPS */
    {
      /*
        airspy_sys_samplerate_t airspy_m4_conf
      */
      {
        /* PLL0AUDIO */
        0x00000000, // uint32_t pll0audio_mdiv;
        0x00000000, // uint32_t pll0audio_npdiv;
        0x00000000, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
        /* IDIVB not used set it to 0 */
        3, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
        { 0, 0, 0 } /* uint8_t padding[3] */
      },
      /* airspy_m0_conf_t airspy_m0_conf */
      {
        1250000, // Freq 5MHz => 2.5MHz IQ => IF Freq = 1.25MHz (r820t_if_freq)
        0, // uint8_t r820t_bw;
        0, // uint8_t padding0;
        0 // uint16_t padding1;
      }
    }
  }, /* End airspy_m0_m4_conf_t airspy_m0_m4_conf[AIRSPY_CONF_M0_M4_NB] */

  /* 2nd Expansion Conf Point M0/M4 ALT Data */ 
  /*
  SampleRate configuration with GP_CLKIN=20MHz
  Configuration of PLL0AUDIO shall not exceed 80MHz for ADCHS
  For PLL0AUDIO sys_clock_samplerate() set SEL_EXT to 1 => MDEC enabled. Fractional divider not used.
  For PLL0AUDIO see UM10503 Rev1.8 "Fig 34. PLL0 with fractional divider" Page 184 / 1420 for more details.
  Configuration of IDIVB => Integer divider B divider values (1/(IDIV + 1))
  */
  /* const airspy_m0_m4_conf_t airspy_m0_m4_alt_conf[AIRSPY_CONF_M0_M4_ALT_NB] = */
  {
    /* Conf 0 => 10 MSPS */
    {
      /*
        CGU_SRC_GP_CLKIN=20MHz from SI5351C CLK7 LPC4370 GP_CLKIN
        airspy_sys_samplerate_t airspy_m4_conf
      */
      {
        /* PLL0AUDIO */
        0x00000000, // uint32_t pll0audio_mdiv;
        0x00000000, // uint32_t pll0audio_npdiv;
        0x00000000, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
        /* IDIVB (from GP_CLKIN) */
        0, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
        { 0, 0, 0 } /* uint8_t padding[3] */
      },
      /* airspy_m0_conf_t airspy_m0_conf */
      {
        5000000, // Freq 20MHz => 10MHz IQ => IF Freq = 5MHz (r820t_if_freq)
        59,  // uint8_t r820t_bw;
        0,// uint8_t padding0;
        0 // uint16_t padding1;
      }
    },
    /* Conf 1 => 2.5 MSPS */
    {
      /*
        airspy_sys_samplerate_t airspy_m4_conf
      */
      {
        /* PLL0AUDIO */
        0x00000000, // uint32_t pll0audio_mdiv;
        0x00000000, // uint32_t pll0audio_npdiv;
        0x00000000, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
        /* IDIVB not used set it to 0 */
        3, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
        { 0, 0, 0 } /* uint8_t padding[3] */
      },
      /* airspy_m0_conf_t airspy_m0_conf */
      {
        1250000, // Freq 5MHz => 2.5MHz IQ => IF Freq = 1.25MHz (r820t_if_freq)
        0, // uint8_t r820t_bw;
        0, // uint8_t padding0;
        0 // uint16_t padding1;
      }
    }
  }, /* End airspy_m0_m4_conf_t airspy_m0_m4_alt_conf[AIRSPY_CONF_M0_M4_ALT_NB] */

  /* 3rd Expansion Conf Point SI5351C Data */ 
  /* si5351c_conf_t si5351c_config[AIRSPY_CONF_SI5351C_NB] = */
  {
    /* Conf 0 (AIRSPY_SI5351C_CONFIG_XTAL) XTAL 25MHZ CLK */
    {
      /* Respective value for register 0 to register 180
      Options->Save device registers (not for factory programming)
      See file Si5351C-AIRSPY_NOS-GM_LPC_20MHz_R820T_20MHz_0pf_New_FULL_NotFactory.txt
      SI5351C Configuration details (XTAL 25MHz):
      Channel 0 = 20MHz R820T (Drive Strength 6mA, Inverted)
      Channel 1 = 32768Hz LPC4370 RTC (Drive Strength 2mA)
      Channel 2 to 6 = Disabled/Not Used
      Channel 7 = 20MHz LPC4370 (Drive Strength 8mA)
      */
      /* conf */
      { 
        0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x8F, 0x01, 0x00, 0x00, // 0 - 9
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x83, 0x83, 0x83, // 10 - 19
        0x83, 0x83, 0x83, 0x4F, 0x00, 0x00, 0x00, 0x05, 0x00, 0x0F, // 20 - 29
        0x99, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x0E, 0x00, 0x00, // 30 - 39
        0x00, 0x00, 0x00, 0x01, 0x00, 0x12, 0x00, 0x00, 0x00, 0x00, // 40 - 49
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 50 - 59
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 60 - 69
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 70 - 79
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 80 - 89
        0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 90 - 99
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 100 - 109
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 110 - 119
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 120 - 129
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 130 - 139
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 140 - 149
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 150 - 159
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 160 - 169
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // 170 - 179
      }
    },
    /* Conf 1 (AIRSPY_SI5351C_CONFIG_CLKIN) AIRSPY_NOS CLKIN 10MHz CLK */
    {
      /* conf */
      {
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x8F, 0x01, 0x00, 0x00, // 0 - 9
        0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x7E, 0x2C, 0x87, 0x87, // 10 - 19
        0x87, 0x87, 0xC7, 0x4F, 0x00, 0x00, 0x00, 0x01, 0x00, 0x2A, // 20 - 29
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x20, 0x00, 0x00, // 30 - 39
        0x00, 0x00, 0x00, 0x01, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, // 40 - 49
        0x04, 0x00, 0x42, 0x86, 0x7F, 0x00, 0x02, 0x80, 0x00, 0x00, // 50 - 59
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 60 - 69
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 70 - 79
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 80 - 89
        0x00, 0x2C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 90 - 99
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 100 - 109
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 110 - 119
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 120 - 129
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 130 - 139
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 140 - 149
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 150 - 159
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 160 - 169
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 170 - 179
      }
    }
  } /* End si5351c_conf_t si5351c_config[AIRSPY_CONF_SI5351C_NB] */
};
