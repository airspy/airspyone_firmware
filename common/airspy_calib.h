/*
 * Copyright 2016 Benjamin Vernoux <bvernoux@airspy.com>
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

#ifndef __AIRSPY_CALIB_H
#define __AIRSPY_CALIB_H

#ifdef __cplusplus
extern "C"
{
#endif

#define AIRSPY_FLASH_CALIB_OFFSET (0x20000) /* After 128KB (Reserved for Firmware + 64KB Spare) */
#define AIRSPY_FLASH_CALIB_HEADER (0xCA1B0001)

/* For each configuration the index corresponds to uint32_t */
typedef struct
{
  uint32_t header; /* Shall be equal to AIRSPY_FLASH_CALIB_HEADER */
  uint32_t timestamp; /* Epoch Unix Time Stamp */
  int32_t correction_ppb;
} airspy_calib_t;

#ifdef __cplusplus
}
#endif

#endif /* __AIRSPY_CALIB_H */
