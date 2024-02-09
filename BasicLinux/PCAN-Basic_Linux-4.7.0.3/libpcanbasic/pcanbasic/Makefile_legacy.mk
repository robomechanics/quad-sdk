# SPDX-License-Identifier: LGPL-2.1-only
#
# PCAN-Basic library Makefile
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
#
# Contact:     <linux@peak-system.com>
# Maintainer:  Stephane Grosjean <s.grosjean@peak-system.com>
# Author:      Thomas Haber <thomas@toem.de>
#
CXX	= $(CROSS_COMPILE)g++
LN	= ln -sf

# Paths and files
SRC     = src
SRC_INC = include

# pcan driver/lib uapi root dir
PCAN_ROOT ?= $(SRC)/pcan

VERSION_FILE = $(SRC)/libpcanbasic_legacy.cpp

INC     = -I. -I$(SRC_INC) -I$(PCAN_ROOT)/driver
FILES   += $(VERSION_FILE)
DBG     = -g
RT      = NO_RT

# get API version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION) | head -1)
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION) | head -1)
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION) | head -1)

ifeq ($(RT), XENOMAI)
#****************************************************************************
# Define flags for XENOMAI installation only
#
SKIN = xeno
RT_DIR          ?= /usr/xenomai
RT_CONFIG       ?= $(RT_DIR)/bin/xeno-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CXXFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

ifeq ($(RT), RTAI)
#****************************************************************************
# Define flags for RTAI installation only
#
SKIN = lxrt
RT_DIR          ?= /usr/realtime
RT_CONFIG       ?= $(RT_DIR)/bin/rtai-config
RT_LIB_DIR      ?= $(shell $(RT_CONFIG) --library-dir) -Wl,-rpath $(shell $(RT_CONFIG) --library-dir)
RT_CXXFLAGS       ?= $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

LIBPATH = $(DESTDIR)/usr/lib

# Define targets
LDNAME  = libpcanbasic.so
SONAME  = $(LDNAME).$(MAJOR)
TARGET  = $(SONAME).$(MINOR).$(PATCH)
SONAME_OLD  = $(LDNAME).0

ifneq ($(RT), NO_RT)
CXXFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc $(INC) -D$(RT) $(RT_CXXFLAGS) -L$(RT_LIB_DIR) -lrtdm
else
CXXFLAGS = -fPIC -shared -O2 -Wall -Wl,-soname,$(SONAME) -lc $(INC) -D$(RT)
endif

#********** entries *********************

all: message $(LDNAME)

$(LDNAME): $(TARGET)
	$(LN) $(TARGET) $(LDNAME)

$(TARGET): $(FILES)
	$(CXX) $(FILES) $(CXXFLAGS) -o $(TARGET)
	$(LN) $(TARGET) $(SONAME)
	$(LN) $(SONAME) $(LDNAME)

clean:
	-rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.* *.so

.PHONY: message
message:
	@echo "***"
	@echo "*** Making PCANBasic library WITHOUT FD support (PCAN driver < 8.0)"
	@echo "***"
	@echo "*** target=$(LDNAME)"
	@echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"

#********** these entries are reserved for root access only *******************
install: $(TARGET)
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME_OLD)
	$(LN) $(LIBPATH)/$(SONAME) $(LIBPATH)/$(LDNAME)
	cp $(SRC_INC)/PCANBasic.h $(DESTDIR)/usr/include/PCANBasic.h
	chmod 644 $(DESTDIR)/usr/include/PCANBasic.h
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif

uninstall:
	-rm $(DESTDIR)/usr/include/PCANBasic.h
	-rm $(LIBPATH)/$(LDNAME)
	-rm $(LIBPATH)/$(SONAME_OLD)
	-rm $(LIBPATH)/$(SONAME)
	-rm $(LIBPATH)/$(TARGET)
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif
