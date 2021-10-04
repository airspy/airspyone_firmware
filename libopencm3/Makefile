##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
## Copyright (C) 2014 Benjamin Vernoux (LPC43xxx) <bvernoux@gmail.com>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

PREFIX		?= arm-none-eabi
#PREFIX		?= arm-elf

ifeq ($(DETECT_TOOLCHAIN),)
DESTDIR		?= /usr/local
else
DESTDIR		?= $(shell dirname $(shell readlink -f $(shell which $(PREFIX)-gcc)))/..
endif

INCDIR		:= $(DESTDIR)/$(PREFIX)/include
LIBDIR		:= $(DESTDIR)/$(PREFIX)/lib
INSTALL		:= install

SRCLIBDIR:= $(realpath lib)

TARGETS:= lpc43xx/m4 lpc43xx/m0 lpc43xx/m0s

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ...".
MAKEFLAGS += --no-print-directory
endif

all: build

build: lib

LIB_DIRS:=$(wildcard $(addprefix lib/,$(TARGETS)))
$(LIB_DIRS):
	@printf "  BUILD   $@\n";
	$(Q)$(MAKE) --directory=$@ SRCLIBDIR=$(SRCLIBDIR)

lib: $(LIB_DIRS)
	$(Q)true

install: lib
	@printf "  INSTALL headers\n"
	$(Q)$(INSTALL) -d $(INCDIR)/libopencm3
	$(Q)$(INSTALL) -d $(INCDIR)/libopencmsis
	$(Q)$(INSTALL) -d $(LIBDIR)
	$(Q)cp -r include/libopencm3/* $(INCDIR)/libopencm3
	$(Q)cp -r include/libopencmsis/* $(INCDIR)/libopencmsis
	@printf "  INSTALL libs\n"
	$(Q)$(INSTALL) -m 0644 lib/*.a $(LIBDIR)
	@printf "  INSTALL ldscripts\n"
	$(Q)$(INSTALL) -m 0644 lib/*.ld $(LIBDIR)

doc:
	$(Q)$(MAKE) -C doc html

clean: $(LIB_DIRS:=.clean) $(EXAMPLE_DIRS:=.clean) doc.clean

%.clean:
	$(Q)if [ -d $* ]; then \
		printf "  CLEAN   $*\n"; \
		$(MAKE) -C $* clean SRCLIBDIR=$(SRCLIBDIR) || exit $?; \
	fi;

.PHONY: build lib $(LIB_DIRS) install doc clean

