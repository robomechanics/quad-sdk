# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - pcaninfo Makefile
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH
#
# Contact: <linux@peak-system.com>
# Author:  Stephane Grosjean <s.grosjean@peak-system.com>
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
#

# Commands
CC	= $(CROSS_COMPILE)gcc
LN	= ln -sf

SRC     = src
PCANBASIC_ROOT = ../pcanbasic

# pcaninfo C default flags
CFLAGS = -O2 -Wall -Wcast-align -Wcast-qual -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wunused

# use -Wshadow with gcc > 4.6 only
#CFLAGS += -Wshadow

# pcaninfo doesn't use libpcanbasic API but compiles with its source files.
# Then, PCAN_ROOT MUST be the same PCAN_ROOT than the one that helped to
# build libpcanbasic.
-include $(PCANBASIC_ROOT)/src/pcan/.config

ifeq ($(CONFIG_PCAN_VERSION),)
PCAN_ROOT := $(shell cd ../..; pwd)
else
PCAN_ROOT = $(PCANBASIC_ROOT)/src/pcan
endif

# libpcanbasic compiles libpcanfd source files
LIBPCANFD_SRC = $(PCAN_ROOT)/lib/src/libpcanfd.c
LIBPCANFD_INC = -I$(PCAN_ROOT)/driver -I$(PCAN_ROOT)/lib

# libpcanfd compile option
RT ?= NO_RT

# pcaninfo source files
FILES   = $(SRC)/main.c
PCANBASIC_SRC = $(PCANBASIC_ROOT)/src
PCANBASIC_INC = $(PCANBASIC_ROOT)/include
FILES   += $(PCANBASIC_SRC)/pcanlog.c
FILES   += $(PCANBASIC_SRC)/pcblog.c
FILES   += $(PCANBASIC_SRC)/pcbtrace.c
FILES   += $(PCANBASIC_SRC)/pcbcore.c
FILES   += $(PCANBASIC_SRC)/pcaninfo.c
FILES   += $(LIBPCANFD_SRC)

# Get build version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
VERSION_FILE = $(SRC)/version.h
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION))
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION))
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION))

# targets
NAME = pcaninfo
EXT = 
TARGET_SHORT = $(NAME)$(EXT)
TARGET  = $(TARGET_SHORT).$(MAJOR).$(MINOR).$(PATCH)
TITLE := $(NAME)

# Define flags for XENOMAI installation only
ifeq ($(RT), XENOMAI)
RT_DIR ?= /usr/xenomai
RT_CONFIG ?= $(RT_DIR)/bin/xeno-config

SKIN := rtdm
RT_CFLAGS := $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
RT_LDFLAGS := -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --skin $(SKIN) --ldflags)
endif

# Define flags for RTAI installation only
ifeq ($(RT), RTAI)
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config

SKIN := lxrt
RT_CFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-cflags)
RT_LDFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-ldflags)
endif

# Complete flags
CFLAGS += -D$(RT) -I$(PCANBASIC_INC) -I$(PCANBASIC_SRC) $(LIBPCANFD_INC) $(RT_CFLAGS)
LDFLAGS += -lm -ldl $(RT_LDFLAGS)

# Installation directory
TARGET_DIR = $(DESTDIR)/usr/local/bin

#********** entries *********************

all: message $(TARGET_SHORT)

$(TARGET_SHORT): $(TARGET)
	$(LN) $(TARGET) $(TARGET_SHORT)

$(TARGET): $(FILES)
	$(CC) $(FILES) $(CFLAGS) $(LDFLAGS) -o $(TARGET)

clean:
	@echo
	@echo "***"
	@echo "*** Cleaning $(TITLE)..."
	@echo "***"
	-rm -f $(SRC)/*~ $(SRC)/*.o $(PCANBASIC_SRC)/*~ $(PCANBASIC_SRC)/*.o *~ *.so.* *.so $(TARGET) $(TARGET_SHORT)

.PHONY: message
message:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE)..."
	@echo "***"
	@echo "*** target=$(NAME)" 
	@echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@echo "*** PCAN_ROOT=$(PCAN_ROOT)"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"
  
xeno:
	$(MAKE) RT=XENOMAI

rtai:
	$(MAKE) RT=RTAI

#********** these entries are reserved for root access only *******************
install: all
	@echo
	@echo "***"
	@echo "*** Installing $(TITLE)..."
	@echo "***"
	cp $(TARGET) $(TARGET_DIR)/$(TARGET_SHORT)
	chmod 755 $(TARGET_DIR)/$(TARGET_SHORT)
  
uninstall:
	@echo
	@echo "***"
	@echo "*** Uninstalling $(TITLE)..."
	@echo "***"
	-rm $(TARGET_DIR)/$(TARGET_SHORT)
