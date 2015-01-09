/** @defgroup i2c_file I2C

@ingroup LPC43xx

@brief <b>libopencm3 LPC43xx I2C</b>

@version 1.0.0

@author @htmlonly &copy; @endhtmlonly 2012 Michael Ossmann <mike@ossmann.com>

LGPL License Terms @ref lgpl_license
*/

/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright (C) 2014 Benjamin Vernoux <bvernoux@gmail.com>
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

/*
 * This is a very minimal I2C driver just to make sure we can get the
 * peripheral working.
 */

/**@{*/

#include <libopencm3/lpc43xx/i2c.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/cgu.h>

#define I2C_TIMEOUT (10000)

#define SFSP_I2C1_SDA_SCL (0x00000001 | SCU_CONF_ZIF_DIS_IN_GLITCH_FILT | SCU_CONF_EZI_EN_IN_BUFFER)

void i2c0_init(const uint16_t duty_cycle_count)
{
  /* enable input on SCL and SDA pins */
  SCU_SFSI2C0 = SCU_I2C0_NOMINAL;

  I2C0_SCLH = duty_cycle_count;
  I2C0_SCLL = duty_cycle_count;

  /* clear the control bits */
  I2C0_CONCLR = (I2C_CONCLR_AAC | I2C_CONCLR_SIC
      | I2C_CONCLR_STAC | I2C_CONCLR_I2ENC);

  /* enable I2C0 */
  I2C0_CONSET = I2C_CONSET_I2EN;
}

void i2c1_init(const uint16_t duty_cycle_count)
{
  /* Configure pin function for I2C1*/
  SCU_SFSP2_3 = SFSP_I2C1_SDA_SCL;
  SCU_SFSP2_4 = SFSP_I2C1_SDA_SCL;

  I2C1_SCLH = duty_cycle_count;
  I2C1_SCLL = duty_cycle_count;

  /* clear the control bits */
  I2C1_CONCLR = (I2C_CONCLR_AAC | I2C_CONCLR_SIC
      | I2C_CONCLR_STAC | I2C_CONCLR_I2ENC);

  /* enable I2C1 */
  I2C1_CONSET = I2C_CONSET_I2EN;
}

/* transmit start bit */
void i2c0_tx_start(void)
{
  uint32_t timeout;

  I2C0_CONCLR = I2C_CONCLR_SIC;
  I2C0_CONSET = I2C_CONSET_STA;

  timeout = 0;
  while( (!(I2C0_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }

  I2C0_CONCLR = I2C_CONCLR_STAC;
}

/* transmit start bit */
void i2c1_tx_start(void)
{
  uint32_t timeout;

  I2C1_CONCLR = I2C_CONCLR_SIC;
  I2C1_CONSET = I2C_CONSET_STA;

  timeout = 0;
  while( (!(I2C1_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }

  I2C1_CONCLR = I2C_CONCLR_STAC;
}

/* transmit data byte */
void i2c0_tx_byte(uint8_t byte)
{
  uint32_t timeout;

  if (I2C0_CONSET & I2C_CONSET_STA)
  {
    I2C0_CONCLR = I2C_CONCLR_STAC;
  }
  I2C0_DAT = byte;
  I2C0_CONCLR = I2C_CONCLR_SIC;

  timeout = 0;
  while( (!(I2C0_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }
}

/* transmit data byte */
void i2c1_tx_byte(uint8_t byte)
{
  uint32_t timeout;

  if (I2C1_CONSET & I2C_CONSET_STA)
  {
    I2C1_CONCLR = I2C_CONCLR_STAC;
  }
  I2C1_DAT = byte;
  I2C1_CONCLR = I2C_CONCLR_SIC;

  timeout = 0;
  while( (!(I2C1_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }
}

/* receive data byte */
uint8_t i2c0_rx_byte(void)
{
  uint32_t timeout;

  if (I2C0_CONSET & I2C_CONSET_STA)
  {
    I2C0_CONCLR = I2C_CONCLR_STAC;
  }
  I2C0_CONCLR = I2C_CONCLR_SIC;

  timeout = 0;
  while( (!(I2C0_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }

  return I2C0_DAT;
}

/* receive data byte (ack=1 => ACK if ack=0 NACK) */
uint8_t i2c1_rx_byte(bool ack)
{
  uint32_t timeout;

  if (I2C1_CONSET & I2C_CONSET_STA)
  {
    I2C1_CONCLR = I2C_CONCLR_STAC;
  }

  if (ack)
  {
    I2C1_CONSET = I2C_CONSET_AA;
  } else
  {
    I2C1_CONCLR = I2C_CONCLR_AAC;
  }

  I2C1_CONCLR = I2C_CONCLR_SIC;
  timeout = 0;
  while( (!(I2C1_CONSET & I2C_CONSET_SI)) && (timeout < I2C_TIMEOUT) )
  {
    timeout++;
  }

  return I2C1_DAT;
}

/* transmit stop bit */
void i2c0_stop(void)
{
  if (I2C0_CONSET & I2C_CONSET_STA) {
    I2C0_CONCLR = I2C_CONCLR_STAC;
  }
  I2C0_CONSET = I2C_CONSET_STO;
  I2C0_CONCLR = I2C_CONCLR_SIC;
}

/* transmit stop bit */
void i2c1_stop(void)
{
  if (I2C1_CONSET & I2C_CONSET_STA) {
    I2C1_CONCLR = I2C_CONCLR_STAC;
  }
  I2C1_CONSET = I2C_CONSET_STO;
  I2C1_CONCLR = I2C_CONCLR_SIC;
}

/**@}*/

