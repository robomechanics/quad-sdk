# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - Build C++ 01_LookUpChannel example for PCAN-Basic
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
#
# Contact:     <linux@peak-system.com>
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
# Author:      Romain Tissier <r.tissier@peak-system.com>
#
CXX ?= $(CROSS_COMPILE)g++

BIN_OUTPUT ?= .

TARGET = $(BIN_OUTPUT)/LookUpChannel
FILES  = $(wildcard *.cpp)

ALL = $(TARGET)

#--- note -------------------------------------------------------------------
# Because these examples have to be built BEFORE libpcanbasic (as well as
# the pcan driver) is installed, local paths HAVE TO be added to the command
# lines of gcc (see usage of -I -L -Wl,rpath in CXXFLAG and LDFLAGS below).
# Once libpcanbasic as well as the pcan driver are installed, these command
# line options are useless and SHOULD NOT be added to the PCAN-Basic
# application compilation command line.
PCANBASIC_ROOT ?= ../../../../pcanbasic

-include $(PCANBASIC_ROOT)/src/pcan/.config

ifeq ($(CONFIG_PCAN_VERSION),)
PCAN_ROOT := $(shell cd $(PCANBASIC_ROOT)/../..; pwd)
else
PCAN_ROOT = $(PCANBASIC_ROOT)/src/pcan
endif

CXXFLAG += -I$(PCANBASIC_ROOT)/include -I$(PCAN_ROOT)/driver -std=c++11
CXXFLAG += -Wno-format-truncation
LDFLAGS += -L$(PCANBASIC_ROOT)/lib -Wl,-rpath $(PCANBASIC_ROOT)/lib

ifeq ($(RT),XENOMAI)
SKIN := alchemy
RT_DIR ?= /usr/xenomai
RT_CONFIG ?= $(RT_DIR)/bin/xeno-config
#CXXFLAG += $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
LDFLAGS += -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --no-auto-init --skin $(SKIN) --ldflags)
endif

ifeq ($(RT),RTAI)
SKIN := lxrt
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config
#CXXFLAG += $(shell $(RT_CONFIG) --$(SKIN)-cflags)
LDFLAGS += $(shell $(RT_CONFIG) --$(SKIN)-ldflags)
endif

#--- end of note ------------------------------------------------------------

LDFLAGS += -lpcanbasic

all: $(ALL)

$(TARGET): $(FILES)
	@mkdir -p $(BIN_OUTPUT)
	$(CXX) $(CXXFLAG) $^ $(LDFLAGS) -o $@

clean:
	-rm -f *.o *~ $(ALL)

# Ignored targets
install uninstall: ;
rtai:
	$(MAKE) RT=RTAI
xeno:
	$(MAKE) RT=XENOMAI
