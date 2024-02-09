# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - PCANBasic JNI Interface
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
# Contact: <linux@peak-system.com>
# Author:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
#

# Commands
CC	= gcc
LN	= ln -sf

# Paths and files
JAVA_HOME = $(shell readlink -f /usr/bin/javac | sed "s:/bin/javac::")
SRC     = src
INC     = -I. -I$(JAVA_HOME)/include/ -I$(JAVA_HOME)/include/linux
FILES   = $(SRC)/pcanbasic_jni.c
LIBPATH = /usr/lib
DEFAULT_PATH_PCANBASIC_HEADER = /usr/include
PWD=$(shell pwd)
# default local PCAN-Basic root (if the library is not installed)
DEFAULT_PATH_PCANBASIC ?=../../libpcanbasic/pcanbasic
PATH_PCANBASIC ?= $(shell [ -d "$(PWD)/$(DEFAULT_PATH_PCANBASIC)" ] && cd $(PWD)/$(DEFAULT_PATH_PCANBASIC); pwd)

# get build version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
VERSION_FILE = 'src/VersionNo.h'
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION))
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION))
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION))

# Define targets
NAME = libpcanbasic_jni
NAME_OLD = libPCANBasic_JNI
EXT = .so
TARGET_SHORT = $(NAME)$(EXT)
TARGET  = $(TARGET_SHORT).$(MAJOR).$(MINOR).$(PATCH)
SONAME  = $(TARGET_SHORT).$(MAJOR)
TITLE := PCANBasic JNI

# User application compile options 
DBG               ?= 
USERAPP_CFLAGS    ?= 
USERAPP_LIBS      ?= -lc -lm -lpcanbasic
USERAPP_LDFLAGS   ?= 

# Check if PCAN-Basic library is installed
PCANBASIC_STATUS = "PCAN-Basic header"
ifeq (,$(wildcard $(DEFAULT_PATH_PCANBASIC_HEADER)/PCANBasic.h))
  PATH_PCANBASIC_HEADER = $(PATH_PCANBASIC)/include
  PCANBASIC_STATUS += "not found, using local path: \'$(PATH_PCANBASIC_HEADER)\'"
  # set local pathes to PCAN-Basic root
  INC 				+= -I$(PATH_PCANBASIC_HEADER)
  USERAPP_CFLAGS	+= -L$(PATH_PCANBASIC)/lib -Wl,-rpath $(PATH_PCANBASIC)
else
  PATH_PCANBASIC_HEADER = $(DEFAULT_PATH_PCANBASIC_HEADER)
  PCANBASIC_STATUS += "found in \'$(PATH_PCANBASIC_HEADER)\'."
endif
PCANBASIC_HEADER = $(PATH_PCANBASIC_HEADER)/PCANBasic.h

# Flags
CFLAGS = -fPIC -shared -O2 -Wl,-soname,$(TARGET_SHORT) $(DBG) $(USERAPP_CFLAGS) $(USERAPP_LIBS) $(INC)
CFLAGS += -Wall -Wcast-align -Wcast-qual -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused


#********** entries *********************

all: message $(TARGET)

$(TARGET) : $(PCANBASIC_HEADER) $(JAVA_HOME) $(FILES) 
	$(CC) $(FILES) $(CFLAGS) -o $(TARGET)

clean:
	@echo
	@echo "***"
	@echo "*** Cleaning $(TITLE)..."
	@echo "***"
	-rm -f $(SRC)/*~ $(SRC)/*.o *~ *.so.* *.so

.PHONY: message
message:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE)..."
	@echo "***"
	@echo "***"
	@echo "*** target=$(TARGET_SHORT)" 
	@echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@echo "***"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"
	@echo "*** JAVA_HOME is '$(JAVA_HOME)'"
	@echo "*** $(PCANBASIC_STATUS)"
	@echo "***"

$(PCANBASIC_HEADER):
	$(error "Failed to locate PCAN-Basic header in '$(PATH_PCANBASIC_HEADER)'")

$(JAVA_HOME):
	$(error "JAVA_HOME does not exist ('$(JAVA_HOME)')")

#********** this entry is reserved for root access only *********************
install: all
	@echo
	@echo "***"
	@echo "*** Installing $(TITLE)..."
	@echo "***"
	cp $(TARGET) $(LIBPATH)/$(TARGET)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	$(LN) $(LIBPATH)/$(SONAME) $(LIBPATH)/$(TARGET_SHORT)
	/sbin/ldconfig
 
uninstall:
	@echo
	@echo "***"
	@echo "*** Uninstalling $(TITLE)..."
	@echo "***"
	-rm $(LIBPATH)/$(TARGET)
	-rm $(LIBPATH)/$(SONAME)
	-rm $(LIBPATH)/$(TARGET_SHORT)	
	/sbin/ldconfig
  
uninstall-purge: uninstall
	-rm $(LIBPATH)/$(TARGET_SHORT).*
	-rm $(LIBPATH)/$(NAME_OLD)$(EXT)*
	/sbin/ldconfig

