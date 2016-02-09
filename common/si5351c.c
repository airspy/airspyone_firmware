/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2014-2016 Benjamin Vernoux <bvernoux@airspy.com>
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

#include "si5351c.h"
#include <libopencm3/lpc43xx/i2c.h>

/*
For configuration only Write Reg 15-92 and 149-170 (in 2 steps).
Like defined in http://community.silabs.com/t5/Silicon-Labs-Knowledge-Base/How-to-program-the-Si5351-via-I2C/ta-p/113755
*/
#define SI5351C_WRITE_CONF_STEP1_REG (15)
#define SI5351C_WRITE_CONF_STEP1_SIZE (78) /* 92 - 15 + 1 */

#define SI5351C_WRITE_CONF_STEP2_REG (149)
#define SI5351C_WRITE_CONF_STEP2_SIZE (22) /* 170 - 149 + 1 */

void si5351c_airspy_config(const si5351c_conf_t* const pt_si5351c_conf)
{
  int i;

  /* Write Conf Step1 */
  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_WRITE);

  i2c0_tx_byte(SI5351C_WRITE_CONF_STEP1_REG);
  for(i = SI5351C_WRITE_CONF_STEP1_REG; i < (SI5351C_WRITE_CONF_STEP1_REG+SI5351C_WRITE_CONF_STEP1_SIZE); i++)
  {
    i2c0_tx_byte(pt_si5351c_conf->conf[i]);
  }
  i2c0_stop();
  
  /* Write Conf Step2 */
  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_WRITE);

  i2c0_tx_byte(SI5351C_WRITE_CONF_STEP2_REG);
  for (i = SI5351C_WRITE_CONF_STEP2_REG; i < (SI5351C_WRITE_CONF_STEP2_REG+SI5351C_WRITE_CONF_STEP2_SIZE); i++)
  {
    i2c0_tx_byte(pt_si5351c_conf->conf[i]);
  }
  i2c0_stop();
}


/* write to single register */
void si5351c_write_single(uint8_t reg, uint8_t val)
{
  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_WRITE);
  i2c0_tx_byte(reg);
  i2c0_tx_byte(val);
  i2c0_stop();
}

/* read single register */
uint8_t si5351c_read_single(uint8_t reg)
{
  uint8_t val;

  /* set register address with write */
  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_WRITE);
  i2c0_tx_byte(reg);

  /* read the value */
  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_READ);
  val = i2c0_rx_byte();
  i2c0_stop();

  return val;
}

/*
 * Write to one or more contiguous registers. data[0] should be the first
 * register number, one or more values follow.
 */
void si5351c_write(uint8_t* const data, const uint_fast8_t data_count)
{
  uint_fast8_t i;

  i2c0_tx_start();
  i2c0_tx_byte(SI5351C_I2C_ADDR | I2C_WRITE);

  for (i = 0; i < data_count; i++)
    i2c0_tx_byte(data[i]);
  i2c0_stop();
}

/* Disable all CLKx outputs. */
void si5351c_disable_all_outputs(void)
{
  uint8_t data[] = { 3, 0xFF };
  si5351c_write(data, sizeof(data));
}

/* Turn off OEB pin control for all CLKx
   Check also the SI5351C work like expected 
   write/read with different pattern
 */
bool si5351c_disable_oeb_pin_control(void)
{
  uint8_t data[] = { 9, 0x00 };
  uint8_t val1;

  /* Check Written Data */
  data[1] = 0x55;
  si5351c_write(data, sizeof(data));
  val1 = si5351c_read_single(9);
  if(val1 == 0x55)
  {
    uint8_t val2;
    uint8_t val3;
    uint8_t val4;

    data[1] = 0x99;
    si5351c_write(data, sizeof(data));
    val2 = si5351c_read_single(9);

    data[1] = 0xAA;
    si5351c_write(data, sizeof(data));
    val3 = si5351c_read_single(9);

    data[1] = 0xFF;
    si5351c_write(data, sizeof(data));
    val4 = si5351c_read_single(9);

    if( (val2 == 0x99) && (val3 == 0xAA) && (val4 == 0xFF) )
    {
      return true;
    }else
    {
      return false;
    }
  }else
  {
    return false;
  }
}

/* Power down all CLKx */
void si5351c_power_down_all_clocks(void)
{
  uint8_t data[] = { 16
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  , SI5351C_CLK_POWERDOWN
  };
  si5351c_write(data, sizeof(data));
}

void si5351c_init_fanout(void)
{
  uint8_t data[] = { 187, 0xc0 };
  si5351c_write(data, sizeof(data));
}

void si5351c_init_xtal(void)
{
  /* Set Crystal Internal CL = 8 pF */
  uint8_t data[] = { 183, 0x92 };
  si5351c_write(data, sizeof(data));
}

void si5351c_init_pll_soft_reset(void)
{
    uint8_t data[] = { 177, 0xac };
    si5351c_write(data, sizeof(data));
}

/* Enable CLK outputs 0(R820T) and 7(LPC) only. */
void si5351c_enable_clock_outputs(void)
{
  uint8_t data[] = { 3, 0x7E };
  si5351c_write(data, sizeof(data));
}

#if 0
/* Enable CLK outputs 0(R820T), 1(RTC) and 7(LPC) only. */
void si5351c_enable_clock_outputs(void)
{
  uint8_t data[] = { 3, 0x7C };
  si5351c_write(data, sizeof(data));
}
#endif
