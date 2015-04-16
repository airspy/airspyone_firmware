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

#ifndef __AIRSPY_USB_REQ_H__
#define __AIRSPY_USB_REQ_H__

#include "usb_type.h"

extern usb_endpoint_t usb_endpoint_bulk_in;
extern usb_endpoint_t usb_endpoint_bulk_out;
extern usb_endpoint_t usb_endpoint_control_in;
extern usb_endpoint_t usb_endpoint_control_out;
extern usb_device_t usb_device;

extern uint32_t sample_rate_conf_no;
extern uint32_t usb_req_set_sample_rate_cmd;

void airspy_usb_req_init(void);

void usb_streaming_disable(void);

#endif//__AIRSPY_USB_REQ_H__
