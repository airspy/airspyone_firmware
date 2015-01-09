/*
 * Copyright 2012 Michael Ossmann <mike@ossmann.com>
 * Copyright 2012 Jared Boone <jared@sharebrained.com>
 * Copyright 2014 Benjamin Vernoux <bvernoux@gmail.com>
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

#ifndef __SI5351C_H
#define __SI5351C_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "airspy_core.h"

#define SI5351C_I2C_ADDR (0x60 << 1)

#define SI5351C_CLK_POWERDOWN (1<<7)

void si5351c_airspy_config(void);

void si5351c_disable_all_outputs(void);
void si5351c_disable_oeb_pin_control(void);
void si5351c_power_down_all_clocks(void);
void si5351c_enable_clock_outputs(void);

void si5351c_init_fanout(void);
void si5351c_init_xtal(void);
void si5351c_init_pll_soft_reset(void);

void si5351c_write_single(uint8_t reg, uint8_t val);
uint8_t si5351c_read_single(uint8_t reg);
void si5351c_write(uint8_t* const data, const uint_fast8_t data_count);

#ifdef __cplusplus
}
#endif

#endif /* __SI5351C_H */
