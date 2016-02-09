# Hey Emacs, this is a -*- makefile -*-
#
# Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
#
# This file is part of AirSpy.
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

BINARY = airspy_m0s

SRC = $(BINARY).c

# Source from ../common dir
VPATH += ../common
SRC += \
	fault_handler.c \
	signal_mcu.c

LDSCRIPT = ../common/LPC4370_M0s_ram_only.ld

include ../common/Makefile_M0s_inc.mk
