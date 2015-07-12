# Hey Emacs, this is a -*- makefile -*-
#
# Copyright 2013 Benjamin Vernoux <bvernoux@sdrsharp.com>
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

#export BOARD ?= AIRSPY_DEMO
#export VERSION_STRING ?= -D'VERSION_STRING="AirSpy DEMO"'

export BOARD ?= AIRSPY_NOS
export VERSION_STRING ?= -D'VERSION_STRING="AirSpy NOS"'

TARGETS = airspy_m0 \
		  airspy_m0s \
		  airspy_rom_to_ram

all: build

build: airspy_fw

airspy_fw:
	$(Q)for i in $(TARGETS); do \
		if [ -d $$i ]; then \
			printf "  BUILD   $$i\n"; \
			$(MAKE) -C $$i || exit $?; \
		fi; \
	done

clean:
	$(Q)for i in $(addprefix lib/,$(TARGETS)) \
		     $(TARGETS); do \
		if [ -d $$i ]; then \
			printf "  CLEAN   $$i\n"; \
			$(MAKE) -C $$i clean || exit $?; \
		fi; \
	done

.PHONY: build airspy_fw
