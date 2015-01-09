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

/* Configuration for M0 core R820T (r820t_set_if_bandwidth() r820t_bw parameter) */
const airspy_m0_conf_t airspy_m0_conf[AIRSPY_CONF_NB] =
{
  /* Conf 0 => AIRSPY_SAMPLERATE_10MSPS = 0 => airspy_samplerate_t */
  {
    5000000, // Freq 20MHz => 10Mhz IQ => IF Freq = 5MHz (r820t_if_freq)
    56  // uint8_t r820t_bw;
  },
  /* Conf 1 => AIRSPY_SAMPLERATE_2_5MSPS = 1 */
  {
    1250000, // Freq 5Mhz => 2.5MHz IQ => IF Freq = 1.25MHz (r820t_if_freq)
    0 // uint8_t r820t_bw;
  }
};
