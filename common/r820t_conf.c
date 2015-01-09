/*
 * Rafael Micro R820T driver for AIRSPY
 *
 * Copyright (C) 2013 Youssef Touil <youssef@airspy.com>
 * Copyright (C) 2013 Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2013 Mauro Carvalho Chehab <mchehab@redhat.com>
 * Copyright (C) 2014 Benjamin Vernoux <bvernoux@gmail.com>
 *
 * This driver is a heavily lifted version of the driver found in the rtlsdr repository:
 * http://cgit.osmocom.org/rtl-sdr/tree/src/tuner_r82xx.c
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <r820t.h>

/* Those initial values start from REG_SHADOW_START */
/* Shall be not set as const as the structure is used as IN & OUT */
r820t_priv_t r820t_conf_rw =
{
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
    /* 10 */ 0x6C,
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
  }
};
