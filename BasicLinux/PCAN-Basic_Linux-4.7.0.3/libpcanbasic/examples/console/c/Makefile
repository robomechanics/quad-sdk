# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - Build C++ examples for PCAN-Basic
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
#
# Contact:     <linux@peak-system.com>
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
# Author:      Thomas Haber <thomas@toem.de>
# Credits:     Klaus Hitschler <klaus.hitschler@gmx.de>
#
CXX = $(CROSS_COMPILE)gcc
SRC = src
RT = NO_RT

TITLE := PCANBasic C examples

# examples applications
TARGET1 = pcanwrite
FILES1  = $(SRC)/$(TARGET1).c 

TARGET2 = pcanread
FILES2  = $(SRC)/$(TARGET2).c 

TARGET3 = pcaneventread
FILES3  = $(SRC)/$(TARGET3).c 

TARGET4 = pcaneventwrite
FILES4  = $(SRC)/$(TARGET4).c 

ALL = $(TARGET1) $(TARGET2) $(TARGET3) $(TARGET4)

#--- note -------------------------------------------------------------------
# Because these examples have to be built BEFORE libpcanbasic (as well as
# the pcan driver) is installed, local paths HAVE TO be added to the command
# lines of gcc (see usage of -I -L -Wl,rpath in CFLAGS and LDFLAGS below).
# Once libpcanbasic as well as the pcan driver are installed, these command
# line options are useless and SHOULD NOT be added to the PCAN-Basic 
# application compilation command line.
PCANBASIC_ROOT ?= ../../../pcanbasic

-include $(PCANBASIC_ROOT)/src/pcan/.config

ifeq ($(CONFIG_PCAN_VERSION),)
PCAN_ROOT := $(shell cd $(PCANBASIC_ROOT)/../..; pwd)
else
PCAN_ROOT = $(PCANBASIC_ROOT)/src/pcan
endif

CFLAGS += -I$(PCANBASIC_ROOT)/include -I$(PCAN_ROOT)/driver
LDFLAGS += -L$(PCANBASIC_ROOT)/lib -Wl,-rpath $(PCANBASIC_ROOT)/lib

#--- end of note ------------------------------------------------------------

# common flags
CFLAGS += -D$(RT)
LDFLAGS += -lpcanbasic

# POSIX comon flags
POSIX_CFLAGS := $(CFLAGS)
POSIX_LDFLAGS := $(LDFLAGS)

# flags for the XENOMAI version
ifeq ($(RT), XENOMAI)
RT_DIR ?= /usr/xenomai
RT_CONFIG ?= $(RT_DIR)/bin/xeno-config

SKIN := alchemy
CFLAGS += $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
LDFLAGS += -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --no-auto-init --skin $(SKIN) --ldflags)

# POSIX flags
POSIX_CFLAGS += $(shell $(RT_CONFIG) --skin posix --cflags)
POSIX_LDFLAGS += -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --no-auto-init --skin posix --ldflags)
endif

# flags for the RTAI version
ifeq ($(RT), RTAI)
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config

SKIN := lxrt
CFLAGS += $(shell $(RT_CONFIG) --$(SKIN)-cflags)
LDFLAGS += $(shell $(RT_CONFIG) --$(SKIN)-ldflags)

# POSIX flags
POSIX_CFLAGS += $(shell $(RT_CONFIG) --$(SKIN)-cflags)
POSIX_LDFLAGS += $(shell $(RT_CONFIG) --$(SKIN)-ldflags)
endif

# some versions of g++ warns about this
CXXFLAG := $(subst -Wstrict-prototypes,,$(CFLAGS))
POSIX_CXXFLAG := $(subst -Wstrict-prototypes,,$(POSIX_CFLAGS))

all: message $(ALL)

message:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE)"
	@echo "***"
	@echo "*** target=$(ALL)" 
	@echo "*** $(CXX) version=$(shell $(CXX) -dumpversion)"
	@echo "*** PCAN_ROOT=$(PCAN_ROOT)"
	@echo "*** PCANBASIC_ROOT=$(PCANBASIC_ROOT)"
	@echo "***"

$(TARGET1): $(FILES1)
	$(CXX) $(CXXFLAG) $^ $(LDFLAGS) -o $@

$(TARGET2): $(FILES2)
	$(CXX) $(CXXFLAG) $^ $(LDFLAGS) -o $@
	
$(TARGET3): $(FILES3)
	$(CXX) $(POSIX_CXXFLAG) $^ $(POSIX_LDFLAGS) -o $@

$(TARGET4): $(FILES4)
	$(CXX) $(POSIX_CXXFLAG) $^ $(POSIX_LDFLAGS) -o $@

xeno:
	$(MAKE) RT=XENOMAI

rtai:
	$(MAKE) RT=RTAI

clean:
	@echo
	@echo "***"
	@echo "*** Cleaning $(TITLE)"
	-rm -f $(SRC)/*~ $(SRC)/*.o *~ $(ALL)

# There is no real need to fill /usr/local/bin with examples
install:
	#cp $(ALL) $(DESTDIR)/usr/local/bin

uninstall:
	#-cd $(DESTDIR)/usr/local/bin; rm -f $(ALL)
