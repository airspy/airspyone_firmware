/*
 * Copyright 2013-2015 Benjamin Vernoux <bvernoux@airspy.com>
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

#ifndef __AIRSPY_CORE_H
#define __AIRSPY_CORE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/lpc43xx/cgu.h>

/* hardware identification number */
#define BOARD_ID_AIRSPY  0
#define BOARD_ID BOARD_ID_AIRSPY

/*
 * SCU PinMux
 */
#define SCU_I2C1_SDA    (P2_3)
#define SCU_I2C1_SCL    (P2_4)

/* GPIO Output PinMux */

#define PIN_EN_LED1     (BIT12) /* GPIO0[12] on P1_17 */
#define PORT_EN_LED1    (GPIO0)

#define PIN_EN_R820T    (BIT7) /* GPIO1[7] on P1_14 */
#define PORT_EN_R820T   (GPIO1)

#define PIN_EN_BIAST    (BIT13) /* GPIO1[13] on P2_13 */
#define PORT_EN_BIAST   (GPIO1)

/* GPIO Input PinMux */
#define SCU_PINMUX_BOOT0    (P1_1)  /* GPIO0[8] on P1_1 */
#define SCU_PINMUX_BOOT1    (P1_2)  /* GPIO0[9] on P1_2 */
#define SCU_PINMUX_BOOT2    (P2_8)  /* GPIO5[7] on P2_8 */
#define SCU_PINMUX_BOOT3    (P2_9)  /* GPIO1[10] on P2_9 */

/* SSP1 Peripheral PinMux not used */
#define SCU_SSP1_MISO       (P1_3)  /* P1_3 */
#define SCU_SSP1_MOSI       (P1_4)  /* P1_4 */
#define SCU_SSP1_SCK        (P1_19) /* P1_19 */
#define SCU_SSP1_SSEL       (P1_20) /* P1_20 */

/* SPI flash */
#define SCU_SSP0_MISO       (P3_6)
#define SCU_SSP0_MOSI       (P3_7)
#define SCU_SSP0_SCK        (P3_3)
#define SCU_SSP0_SSEL       (P3_8) /* GPIO5[11] on P3_8 */
#define SCU_FLASH_HOLD      (P3_4) /* GPIO1[14] on P3_4 */
#define SCU_FLASH_WP        (P3_5) /* GPIO1[15] on P3_5 */

/* TODO add other Pins */

/*
 * GPIO Pins
 */

/* GPIO Output */

#define PIN_FLASH_HOLD (BIT14) /* GPIO1[14] on P3_4 */
#define PIN_FLASH_WP   (BIT15) /* GPIO1[15] on P3_5 */
#define PORT_FLASH     (GPIO1)
#define PIN_SSP0_SSEL  (BIT11) /* GPIO5[11] on P3_8 */
#define PORT_SSP0_SSEL (GPIO5)

/* GPIO Input */
#define PIN_BOOT0   (BIT8)  /* GPIO0[8] on P1_1 */
#define PIN_BOOT1   (BIT9)  /* GPIO0[9] on P1_2 */
#define PIN_BOOT2   (BIT7)  /* GPIO5[7] on P2_8 */
#define PIN_BOOT3   (BIT10) /* GPIO1[10] on P2_9 */

/* Read GPIO Pin */
#define GPIO_STATE(port, pin) ((GPIO_PIN(port) & (pin)) == (pin))
#define BOOT0_STATE       GPIO_STATE(GPIO0, PIN_BOOT0)
#define BOOT1_STATE       GPIO_STATE(GPIO0, PIN_BOOT1)
#define BOOT2_STATE       GPIO_STATE(GPIO5, PIN_BOOT2)
#define BOOT3_STATE       GPIO_STATE(GPIO1, PIN_BOOT3)

/* TODO add other Pins */

/* Params for pll0audio_ctrl_flags can be ORed together */
#define PLL0AUDIO_CTRL_FLAG_DIRECT_I  (CGU_PLL0AUDIO_CTRL_DIRECTI)
#define PLL0AUDIO_CTRL_FLAG_DIRECT_O  (CGU_PLL0AUDIO_CTRL_DIRECTO)

/* Params for pll0usb_ctrl_flags can be ORed together */
#define PLL0USB_CTRL_FLAG_DIRECT_I  (CGU_PLL0USB_CTRL_DIRECTI)
#define PLL0USB_CTRL_FLAG_DIRECT_O  (CGU_PLL0USB_CTRL_DIRECTO)

typedef struct
{
  /* Use PLL0AUDIO or IDIVB for ADCHS samplerate (with CGU_SRC_GP_CLKIN as source) */
  /* 
    1) If pll0audio_mdiv=0 & pll0audio_npdiv=0 then use IDIVB "idivb" for ADCHS.
     1-1) If idivb = 0, Do not use IDIVB direct connection CGU_SRC_GP_CLKIN to ACHS_CLK.
    2) Else use PLLAudio => pll0audio_mdiv & pll0audio_npdiv for ADCHS
  */
  /* PLL0AUDIO configuration for ADCHS clock 
    (PLL Register settings (DirectI) Normal PLL Usecase(SEL_EXT=1)
  */
  uint32_t pll0audio_mdiv;
  uint32_t pll0audio_npdiv;
  uint32_t pll0audio_ctrl_flags;
  /* IDIVB configuration for ADCHS clock */
  uint8_t idivb; /* 0 to 15 */
  uint8_t padding[3]; /* padding for struct align on 32bits */
} airspy_sys_samplerate_t;

/* SI5351C configurable registers */
typedef struct
{
  uint8_t reg_index;
  uint8_t reg_value;
} airspy_si5351c_reg_t;

#define AIRSPY_SI5351C_CONF_NB_REG (6)

typedef struct
{
  airspy_si5351c_reg_t reg[AIRSPY_SI5351C_CONF_NB_REG];
} airspy_si5351c_t;

/* PLL1 High Speed Conf */
typedef struct
{
  uint32_t pll1_hs_psel;
  uint32_t pll1_hs_nsel;
  uint32_t pll1_hs_msel;
} airspy_pll1_hs_t;

/* PLL1 Low Speed Conf */
typedef struct
{
  uint32_t pll1_ls_psel;
  uint32_t pll1_ls_nsel;
  uint32_t pll1_ls_msel;
} airspy_pll1_ls_t;

/*
ADCHS => IDIVB => IDIV (0 to 15)
SI5351C => Channel7 LPC4370 CLK_IN => X regs TBD
I2C SI5351C => I2C0 Init val 127 => Example 400kHz (100MHz PLL1/(2*127)=0.394MHz)
I2C R820T => I2C1 Init val 127 => Example 400kHz (100MHz PLL1/(2*127)=0.394MHz)
PLL1 (M4/M0 core, Peripheral, APB1) => PSEL, NSEL, MSEL
USB HS => PLL0USB => MDIV, NP_DIV (2x32bits)
*/
typedef struct
{
  /* PLL0USB */
  uint32_t pll0_usb_mdiv;
  uint32_t pll0_usb_npdiv;
  uint32_t pll0usb_ctrl_flags;
  /* PLL1 High Speed Mode */
  airspy_pll1_hs_t pll1_hs;
  /* PLL1 Low Speed Mode */
  airspy_pll1_ls_t pll1_ls;
  /* ADCHS samplerate */
  airspy_sys_samplerate_t airspy_sys_samplerate;
} airspy_sys_clock_t;

/* Multi Core order (conf/command...) */
#define SET_SAMPLERATE_CMD  (1)
#define START_ADCHS_CMD  (1)
#define STOP_ADCHS_CMD   (2)

typedef struct
{
  union
  {
    struct
    {
      uint8_t cmd; /* Up to 255 commands (0 is reserved) */
      uint8_t conf; /* Up to 256 configuration (example samplerate ...) */
      uint16_t spare; /* Spare for alignment and for future use */
    };
    uint32_t raw;
  };
} airspy_mcore_t;

void delay(uint32_t duration);

void cpu_reset(void);

void sys_clock_init(const airspy_sys_clock_t* const pt_airspy_sys_conf);

void sys_clock_samplerate(const airspy_sys_samplerate_t* const pt_airspy_sys_conf);

void cpu_clock_pll1_high_speed(const airspy_pll1_hs_t* const pt_airspy_pll1_hs_conf);

void cpu_clock_pll1_low_speed(const airspy_pll1_ls_t* const pt_airspy_pll1_ls_conf);

void pin_setup(void);

void led_on(void);
void led_off(void);

bool sample_rate_frac_set(uint32_t rate_num, uint32_t rate_denom);
bool sample_rate_set(const uint32_t sampling_rate_hz);

void enable_r820t_power(void);
void disable_r820t_power(void);

void enable_biast_power(void);
void disable_biast_power(void);


#ifdef __cplusplus
}
#endif

#endif /* __AIRSPY_CORE_H */
