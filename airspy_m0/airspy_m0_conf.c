/*
* Copyright (c) 2014, 2015 Benjamin Vernoux <bvernoux@airspy.com>
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

// Configuration for M0 core R820T (r820t_set_if_bandwidth() r820t_bw parameter)
const airspy_m0_conf_t airspy_m0_conf[AIRSPY_CONF_NB] = {
  {// Conf 0 => 20MSPS = AIRSPY_SAMPLERATE_10MSPS
    5000000, // Freq 20MHz => 10Mhz IQ => IF Freq = 5.00MHz (r820t_if_freq)
    59       // uint8_t r820t_bw;
  },
  {// Conf 1 => 5MSPS = AIRSPY_SAMPLERATE_2_5MSPS
    1250000, // Freq 5Mhz => 2.5MHz IQ => IF Freq = 1.25MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  },
  {// Conf 2 => 40MSPS = AIRSPY_SAMPLERATE_20MSPS
    10000000, // Freq 40Mhz => 20MHz IQ => IF Freq = 10.00MHz (r820t_if_freq)
    59        // uint8_t r820t_bw;
  },
  {// Conf 3 => 32MSPS = AIRSPY_SAMPLERATE_16MSPS
    8000000, // Freq 32Mhz => 16MHz IQ => IF Freq = 8.00MHz (r820t_if_freq)
    59       // uint8_t r820t_bw;
  },
  {// Conf 4 => 24MSPS = AIRSPY_SAMPLERATE_12MSPS
    6000000,  // Freq 24Mhz => 12MHz IQ => IF Freq = 6.00MHz (r820t_if_freq)
    59        // uint8_t r820t_bw;
  },
  {// Conf 5 => 16MSPS = AIRSPY_SAMPLERATE_8_0MSPS
    4000000, // Freq 16Mhz => 8.0MHz IQ => IF Freq = 4.00MHz (r820t_if_freq)
    59       // uint8_t r820t_bw;
  },
  {// Conf 6 => 12MSPS = AIRSPY_SAMPLERATE_6_0MSPS
    3000000, // Freq 12Mhz => 6MHz IQ => IF Freq = 3.00MHz (r820t_if_freq)
    59       // uint8_t r820t_bw;
  },
  {// Conf 7 => 10MSPS = AIRSPY_SAMPLERATE_5_0MSPS
    2500000, // Freq 10Mhz => 5.0MHz IQ => IF Freq = 2.50MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  },
  {// Conf 8 => 8MSPS = AIRSPY_SAMPLERATE_4_0MSPS
    2000000, // Freq 8Mhz => 4.0MHz IQ => IF Freq = 2.00MHz (r820t_if_freq)
    59        // uint8_t r820t_bw;
  },
  {// Conf 9 => 6MSPS = AIRSPY_SAMPLERATE_3_0MSPS
    1500000, // Freq 6Mhz => 3MHz IQ => IF Freq = 1.50MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  },
  {// Conf 10 => 4.8MSPS = AIRSPY_SAMPLERATE_2_4MSPS
    1200000, // Freq 4.8Mhz => 2.4MHz IQ => IF Freq = 1.2MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  },
  {// Conf 11 => 4MSPS = AIRSPY_SAMPLERATE_2_0MSPS
    1000000, // Freq 4Mhz => 2.0MHz IQ => IF Freq = 1.00MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  },
  {// Conf 12 => 2MSPS = AIRSPY_SAMPLERATE_1_0MSPS
    500000,  // Freq 2Mhz => 1.0MHz IQ => IF Freq = 0.50MHz (r820t_if_freq)
    0        // uint8_t r820t_bw;
  }
};
