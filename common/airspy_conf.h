/*
 * Copyright 2014 Benjamin Vernoux <bvernoux@airspy.com>
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

#ifndef __AIRSPY_CONF_H
#define __AIRSPY_CONF_H

#include "airspy_core.h"
#include "airspy_commands.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define AIRSPY_SI5351C_CONFIG_XTAL (0)
#define AIRSPY_SI5351C_CONFIG_CLKIN (1)

#ifdef USE_PACKING
/* Configure I2C0 & IC21 to about 400kHz (180MHz/(2*280)=0.32MHz) */
#define AIRSPY_I2C0_PLL1_LS_HS_CONF_VAL (280)
#define AIRSPY_I2C1_PLL1_HS_CONF_VAL (280)
#else
/* Configure I2C0 & IC21 to 400kHz (140MHz/(2*175)=0.4MHz) */
#define AIRSPY_I2C0_PLL1_LS_HS_CONF_VAL (175)
#define AIRSPY_I2C1_PLL1_HS_CONF_VAL (175)
#endif

/* Configure I2C1 to 400kHz (40MHz/(2*50)=0.4MHz) */
#define AIRSPY_I2C1_PLL1_LS_CONF_VAL (50)

#define AIRSPY_SAMPLERATE_DEFAULT_CONF (0)

 /* See airspy_commands.h number of items in uint32_t */
#define AIRSPY_CONF_NB (2)

#define AIRSPY_SAMPLERATE_CMD_SHIFT_BIT (3) // Up to 8 conf

#ifdef USE_PACKING
/* 0=No Packing, 1=Packing Enabled */
#define AIRSPY_PACKING (1) 
#else
#define AIRSPY_PACKING (0) 
#endif

/* For each configuration the index corresponds to uint32_t */
typedef struct
{
  uint32_t r820t_if_freq;
  uint8_t r820t_if_bw; /* from 0 to 63 */
} airspy_m0_conf_t;

//
// M4 adds data to q[q_head]
// M0 removes data from q[q_tail]
//
#define M0_QUEUE_SIZE (4)
#define M0_QUEUE_MASK (M0_QUEUE_SIZE-1)
typedef struct {
  uint32_t q_head;
  uint32_t q_tail;
  uint32_t q[M0_QUEUE_SIZE];
} airspy_m0_queue_t;

/* Configuration for M4 core see airspy_conf_m4.c */
extern const airspy_sys_clock_t airspy_m4_init_conf;
extern const airspy_sys_samplerate_t airspy_m4_conf[AIRSPY_CONF_NB];

/* Configuration for M0 core (r820t_set_if_bandwidth() r820t_bw parameter) see airspy_conf_m0.c */
extern const airspy_m0_conf_t airspy_m0_conf[AIRSPY_CONF_NB];

/* Configuration for SI5351C */
extern uint8_t si5351c_config[2][180];

#ifdef __cplusplus
}
#endif

#endif /* __AIRSPY_CONF_H */
