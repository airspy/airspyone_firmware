# Hey Emacs, this is a -*- makefile -*-
#
# Copyright 2012 Jared Boone <jared@sharebrained.com>
# Copyright 2013/2014 Benjamin Vernoux <bvernoux@gmail.com>
#
# This file is part of AirSpy (based on HackRF project).
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
#

BINARY = airspy_m0

SRC = $(BINARY).c\
	airspy_rx.c \
	airspy_usb_req.c \
	usb_descriptor.c \
	usb_device.c \
	usb_endpoint.c \
	airspy_m0_conf.c

# Source from ../common dir
VPATH += ../common
SRC += \
	airspy_core.c \
	fault_handler.c \
	si5351c.c \
	r820t.c \
	r820t_conf.c \
	w25q80bv.c \
	rom_iap.c \
	signal_mcu.c \
	usb.c \
	usb_queue.c \
	usb_request.c \
	usb_standard_request.c

LDSCRIPT = ../common/LPC4370_M0_ram_only.ld

include ../common/Makefile_M0_inc.mk
