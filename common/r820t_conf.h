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

#ifndef __R820T_CONF_H
#define __R820T_CONF_H

#include "r820t.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Shall be not set as const as the structure is used as IN & OUT */
extern r820t_priv_t r820t_conf_rw;

#ifdef __cplusplus
}
#endif

#endif /* __R820T_CONF_H */
