/*
 * Copyright 2014 Benjamin Vernoux <bvernoux@gmail.com>
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
/*
  WARNING PLL1 CONF for I2C0 & I2C1 from airspy_conf.h shall be modified, 
  according to PLL1 High Speed Mode & PLL1 Low Speed Mode defined below
*/

/*
Clock configuration with GP_CLKIN=20MHz
Configuration of clocks at init (sys_clock_init())
*/
const airspy_sys_clock_t airspy_m4_init_conf =
{
  /* Configure PLL0USB to produce 480MHz clock from GP_CLKIN */
  0x040E03FF, // uint32_t pll0_usb_mdiv;
  0x00000000, // uint32_t pll0_usb_npdiv;
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
  },
  {
    /* ADCHS samplerate */
    /* PLL0AUDIO (from GP_CLKIN) */
    0, // uint32_t pll0audio_mdiv;
    0, // uint32_t pll0audio_npdiv;
    PLL0AUDIO_CTRL_FLAG_DIRECT_I, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
    /* IDIVB (from 20MHz GP_CLKIN) not used set it to 0 */
    0, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
    { 0, 0, 0 } /* uint8_t padding[3] */
  },
};
 
/*
SampleRate configuration with GP_CLKIN=20MHz
Configuration of PLL0AUDIO shall not exceed 80MHz for ADCHS
For PLL0AUDIO sys_clock_samplerate() set SEL_EXT to 1 => MDEC enabled. Fractional divider not used.
For PLL0AUDIO see UM10503 Rev1.8 "Fig 34. PLL0 with fractional divider" Page 184 / 1420 for more details.
Configuration of IDIVB => Integer divider B divider values (1/(IDIV + 1))
*/
const airspy_sys_samplerate_t airspy_m4_conf[AIRSPY_CONF_NB] =
{
  /* Conf 0 => AIRSPY_SAMPLERATE_10MSPS = 0 => airspy_samplerate_t
  CGU_SRC_GP_CLKIN=20MHz from SI5351C CLK7 LPC4370 GP_CLKIN (see airspy_m4_init_conf)
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
  /* Conf 1 => AIRSPY_SAMPLERATE_2_5MSPS = 1 */
  {
    /* PLL0AUDIO */
    0x00000000, // uint32_t pll0audio_mdiv;
    0x00000000, // uint32_t pll0audio_npdiv;
    0x00000000, // uint32_t pll0audio_ctrl_flags; DirectI=PLL0AUDIO_CTRL_FLAG_DIRECT_I or/and DirectO=PLL0AUDIO_CTRL_FLAG_DIRECT_O */
    /* IDIVB not used set it to 0 */
    3, // uint8_t adchs_idivb; /* 0 to 15 (0 means direct connection GP_CLKIN to ADCHS_CLK) */
    { 0, 0, 0 } /* uint8_t padding[3] */
  }
};
 
uint8_t si5351c_config[180] =
{
#ifdef AIRSPY_NOS
  /* Respective value for register 0 to register 180
  Options->Save device registers (not for factory programming)
  See file Si5351C-AIRSPY_NOS-GM_LPC_20MHz_R820T_20MHz_0pf_New_FULL_NotFactory.txt
  SI5351C Configuration details (XTAL 25MHz):
  Channel 0 = 20MHz R820T (Drive Strength 6mA, Inverted)
  Channel 1 = 32768Hz LPC4370 RTC (Drive Strength 2mA)
  Channel 2 to 6 = Disabled/Not Used
  Channel 7 = 20MHz LPC4370 (Drive Strength 8mA)
  */
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
 
#else /* AIRSPY One/Demo */
  /* Respective value for register 0 to register 180
  Options->Save device registers (not for factory programming)
  See file Si5351C-AIRSPY_NOS-GM_LPC_20MHz_R820T_20MHz_0pf_New_FULL_NotFactory.txt
  SI5351C Configuration details (XTAL 25MHz):
  Channel 0 = 20MHz R820T (Drive Strength 2mA)
  Channel 1 = 32768Hz LPC4370 RTC (Drive Strength 2mA)
  Channel 2 to 6 = Disabled/Not Used
  Channel 7 = 20MHz LPC4370 (Drive Strength 2mA)
  */
  0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // 0 - 9
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x2C, 0x83, 0x83, // 10 - 19
  0x83, 0x83, 0x83, 0x4C, 0x00, 0x00, 0x00, 0x05, 0x00, 0x0F, // 20 - 29
  0x99, 0x00, 0x00, 0x03, 0x00, 0x01, 0x00, 0x0A, 0x00, 0x00, // 30 - 39
  0x00, 0x00, 0x00, 0x01, 0x00, 0x0D, 0x00, 0x00, 0x00, 0x00, // 40 - 49
  0x04, 0x00, 0x42, 0x3A, 0x34, 0x00, 0x01, 0x80, 0x00, 0x00, // 50 - 59
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
#endif
};

