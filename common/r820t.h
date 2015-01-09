/*
 * Copyright 2013 Benjamin Vernoux <bvernoux@gmail.com>
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

#ifndef __R820T_H
#define __R820T_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "airspy_commands.h"

#define R820T_I2C_ADDR (0x1A << 1)

#define REG_SHADOW_START 5
#define NUM_REGS 30

/* R820T Clock */
#define XTAL_FREQ_HZ 20000000
#define CALIBRATION_LO 88000

typedef struct
{
  uint32_t freq;
  uint32_t if_freq;
  uint8_t regs[NUM_REGS];
} r820t_priv_t;

void airspy_r820t_write_single(r820t_priv_t *priv, uint8_t reg, uint8_t val);
uint8_t airspy_r820t_read_single(r820t_priv_t *priv, uint8_t reg);

int r820t_init(r820t_priv_t *priv, const uint32_t if_freq);
int r820t_set_freq(r820t_priv_t *priv, uint32_t freq);
int r820t_set_lna_gain(r820t_priv_t *priv, uint8_t gain_index);
int r820t_set_mixer_gain(r820t_priv_t *priv, uint8_t gain_index);
int r820t_set_vga_gain(r820t_priv_t *priv, uint8_t gain_index);
int r820t_set_lna_agc(r820t_priv_t *priv, uint8_t value);
int r820t_set_mixer_agc(r820t_priv_t *priv, uint8_t value);
void r820t_set_if_bandwidth(r820t_priv_t *priv, uint8_t bw);

#ifdef __cplusplus
}
#endif

#endif /* __R820T_H */
