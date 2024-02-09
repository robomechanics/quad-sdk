# SPDX-License-Identifier: LGPL-2.1-only
#
# PCAN-Basic library Makefile (for pcan >= v8.x)
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
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
#

# Commands
CC = $(CROSS_COMPILE)gcc
INSTALL = install
LN = ln -sf
ECHO = /bin/echo

# libpcanbasic C default flags
OPTIMIZATION ?= "-O2"
CFLAGS = -fPIC -shared $(OPTIMIZATION)
CFLAGS += -Wall -Wcast-align -Wcast-qual -Wimplicit 
CFLAGS += -Wpointer-arith -Wswitch
CFLAGS += -Wredundant-decls -Wreturn-type -Wunused

# use -Wshadow with gcc > 4.6 only
#CFLAGS += -Wshadow

# Paths and files
SRC = src
SRC_INC = include
OUT = out
LIB = lib
OUT32 = $(OUT)32
LIB32 = $(LIB)32

# pcan driver/lib uapi root dir
PCAN_ROOT ?= $(SRC)/pcan

# libpcanbasic compiles libpcanfd source files
LIBPCANFD_SRC = $(PCAN_ROOT)/lib/src/libpcanfd.c
LIBPCANFD_INC = -I$(PCAN_ROOT)/driver -I$(PCAN_ROOT)/lib
LIBPCANFD_OBJ = libpcanfd.o

# libpcanfd compile option 
RT ?= NO_RT

# libpcanbasis cource files
FILES   = $(SRC)/libpcanbasic.c
FILES   += $(SRC)/pcaninfo.c
FILES   += $(SRC)/pcanlog.c
FILES   += $(SRC)/pcbcore.c
FILES   += $(SRC)/pcblog.c
FILES   += $(SRC)/pcbtrace.c
ALL_OBJ :=  $(foreach f,$(FILES),$(OUT)/$(basename $(notdir $(f))).o)

# get build version
SED_GET_VERSION = 's/^\#.*[\t\f ]+([0-9]+)[\t\f \r\n]*/\1/'
VERSION_FILE = 'src/version.h'
MAJOR = $(shell cat $(VERSION_FILE) | grep VERSION_MAJOR | sed -re $(SED_GET_VERSION))
MINOR = $(shell cat $(VERSION_FILE) | grep VERSION_MINOR | sed -re $(SED_GET_VERSION))
PATCH = $(shell cat $(VERSION_FILE) | grep VERSION_PATCH | sed -re $(SED_GET_VERSION))

# targets
NAME = libpcanbasic
EXT = .so
TARGET_SHORT = $(NAME)$(EXT)
TARGET = $(TARGET_SHORT).$(MAJOR).$(MINOR).$(PATCH)
SONAME = $(TARGET_SHORT).$(MAJOR)
SONAME_OLD = $(TARGET_SHORT).0
TITLE := PCANBasic library

# Define flags for XENOMAI installation only
ifeq ($(RT), XENOMAI)
RT_DIR ?= /usr/xenomai
RT_CONFIG ?= $(RT_DIR)/bin/xeno-config

SKIN := rtdm
RT_CFLAGS := $(shell $(RT_CONFIG) --skin $(SKIN) --cflags)
RT_LDFLAGS := -Wl,-rpath $(shell $(RT_CONFIG) --library-dir) $(shell $(RT_CONFIG) --skin $(SKIN) --ldflags --auto-init-solib)
endif

# Define flags for RTAI installation only
ifeq ($(RT), RTAI)
RT_DIR ?= /usr/realtime
RT_CONFIG ?= $(RT_DIR)/bin/rtai-config

SKIN := lxrt
RT_CFLAGS := $(shell $(RT_CONFIG) --$(SKIN)-cflags)
endif

# User defined extra flags
EXTRA_CFLAGS      ?= 
EXTRA_LIBS        ?= 
EXTRA_LDFLAGS     ?= 

# Complete flags
CFLAGS += -D$(RT) -I$(SRC_INC) $(LIBPCANFD_INC) $(RT_CFLAGS) $(EXTRA_CFLAGS)
LDFLAGS_CLEANUP_SO ?=
# Note: stripping symbols in a shared library can output warnings on ARM with Binutils <2.33 (temporarily disabled)
#LDFLAGS_CLEANUP_SO += -Xlinker -fvisibility=hidden -Xlinker --retain-symbols-file=$(SRC)/libpcanbasic.def
LDFLAGS += -lm -lpthread $(RT_LDFLAGS) $(EXTRA_LDFLAGS) $(EXTRA_LIBS) $(LDFLAGS_CLEANUP_SO)

ALL := $(TARGET_SHORT)

# multiarch compilation available only in non RT
ifeq ($(RT),NO_RT)
# test if running compiler is able to build 32-bit applications.
# In order to know what kind of executable the linker builds:
# $ ld --print-output-format
GCC_32OPT := $(shell $(CC) -print-multi-lib | awk -F ";" '/^32/ { sub("@","-",$$2); print $$2 }')
ifneq ($(GCC_32OPT),)
# Running a 64-bit host needs to build the 32-bit libpcanbasic too.
# Test if building 32-bit libpcanbasic is possible:
define LIBC32_NOK
$(shell $(ECHO) -e "
#include <sys/cdefs.h>\n
#include <errno.h>\n
int main() { return 0; }" | $(CC) $(GCC_32OPT) -o /dev/null -x c - 2>&1)
endef
ifeq ($(LIBC32_NOK),)
ALL32 := message32 $(foreach a,$(ALL),$(LIB32)/$(a))
CFLAGS32 = $(GCC_32OPT) # -m32
LDFLAGS32 = $(GCC_32OPT) # -m32
else
#$(error from:$(LIBC32_NOK):to)
ALL32 = no_i386_libc
# compiler is able to build 32-bit binaries but system lacks of any 32-bit
# libc. For example:
# # apt-get install libc6-dev-i386[-cross]
endif
else
# compiler ISNOT able to build any 32-bit binary.
ALL32 = no_m32_compiler
endif
endif

# Installation directory
LIBPATH = $(DESTDIR)/usr/lib
LIB32PATH = $(LIBPATH)32

#********** entries *********************

all: message $(foreach a,$(ALL),$(LIB)/$(a)) $(ALL32)

$(LIB)/$(TARGET_SHORT): $(LIB)/$(TARGET)
	cd $(dir $@); $(LN) $(TARGET) $(TARGET_SHORT)

$(LIB)/$(TARGET): $(ALL_OBJ) $(OUT)/$(LIBPCANFD_OBJ)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(TARGET_SHORT) -o $@ $^ $(LDFLAGS)

$(OUT)/$(LIBPCANFD_OBJ): $(LIBPCANFD_SRC)
	$(CC) $(CFLAGS) -c $< -o $@

$(OUT)/%.o: $(SRC)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean
clean:
	@echo
	@echo "***"
	@echo "*** Cleaning $(TITLE)..."
	@echo "***"
	-rm -f $(SRC)/*~ $(OUT)/*.o $(OUT)/*.gcno *~ *.so.* *.so $(LIB)/*~ $(LIB)/*.so.* $(LIB)/*.so $(LIB32)/*~ $(LIB32)/*.so.* $(LIB32)/*.so $(OUT32)/*.o $(OUT32)/*.gcno
	@-rmdir $(OUT) $(OUT32) $(LIB) $(LIB32) 2> /dev/null ||:


ifneq ($(CFLAGS32),)
ALLOBJ32 := $(foreach f,$(FILES),$(OUT32)/$(basename $(notdir $(f))).o)

$(LIB32)/$(TARGET_SHORT): $(LIB32)/$(TARGET)
	cd $(dir $@); $(LN) $(TARGET) $(TARGET_SHORT)
	
$(LIB32)/$(TARGET): $(ALLOBJ32) $(OUT32)/$(LIBPCANFD_OBJ)
	@mkdir -p $(dir $@)
	$(CC) -shared -Wl,-soname,$(SONAME) -o $@ $^ $(LDFLAGS) $(LDFLAGS32)

$(OUT32)/$(LIBPCANFD_OBJ): $(LIBPCANFD_SRC)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CFLAGS32) -c $< -o $@
	
$(OUT32)/%.o: $(SRC)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CFLAGS32) -c $< -o $@
else
BUILD_ARCH_IS_64=$(shell $(CC) -dumpmachine | grep 64)
.PHONY: no_i386_libc no_m32_compiler
no_i386_libc:
ifneq ($(BUILD_ARCH_IS_64),)
	@echo
	@echo "Info: only 64-bit version of libpcanbasic can be built. 32-bit version of libpcanbasic will not be build because the 32-bit version of libc seems not being installed..."
	@echo
endif

no_m32_compiler:
ifneq ($(BUILD_ARCH_IS_64),)
	@echo
	@echo "Info: only 64-bit version of libpcanbasic can be built. 32-bit version of libpcanbasic will not be build because 64-bit compiler is not able to build any 32-bit binary."
	@echo
endif
endif


.PHONY: message message32 
message:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE) with FD support (PCAN driver >= 8.0)..."
	@echo "***"
	@echo "*** target=$(NAME)" 
	@echo "*** version=$(MAJOR).$(MINOR).$(PATCH)"
	@echo "*** PCAN_ROOT=$(PCAN_ROOT)"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"
	mkdir -p $(OUT)
	
message32:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE) (32 bit version)..."
	@echo "***"	
	mkdir -p $(OUT32)
		
	
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
	$(INSTALL) $(LIB)/$(TARGET) $(LIBPATH)/$(TARGET)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME)
	$(LN) $(LIBPATH)/$(TARGET) $(LIBPATH)/$(SONAME_OLD)
	$(LN) $(LIBPATH)/$(SONAME) $(LIBPATH)/$(TARGET_SHORT)
	if [ -d $(LIB32) ]; then \
		mkdir -p $(LIB32PATH); \
		$(INSTALL) $(LIB32)/$(TARGET) $(LIB32PATH)/$(TARGET); \
		$(LN) $(LIB32PATH)/$(TARGET) $(LIB32PATH)/$(SONAME); \
		$(LN) $(LIB32PATH)/$(TARGET) $(LIB32PATH)/$(SONAME_OLD); \
		$(LN) $(LIB32PATH)/$(SONAME) $(LIB32PATH)/$(TARGET_SHORT); \
	fi
	$(INSTALL) $(SRC_INC)/PCANBasic.h $(DESTDIR)/usr/include/PCANBasic.h
	chmod 644 $(DESTDIR)/usr/include/PCANBasic.h
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif
  
uninstall:
	@echo
	@echo "***"
	@echo "*** Uninstalling $(TITLE)..."
	@echo "***"
	-rm $(DESTDIR)/usr/include/PCANBasic.h
	-rm $(LIBPATH)/$(TARGET_SHORT)
	-rm $(LIBPATH)/$(SONAME_OLD)
	-rm $(LIBPATH)/$(SONAME)
	-rm $(LIBPATH)/$(TARGET)
	
	-rm $(LIB32PATH)/$(TARGET_SHORT)
	-rm $(LIB32PATH)/$(SONAME_OLD)
	-rm $(LIB32PATH)/$(SONAME)
	-rm $(LIB32PATH)/$(TARGET)
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif

uninstall-purge: uninstall
	-rm $(LIBPATH)/$(TARGET_SHORT).*
	-rm $(LIB32PATH)/$(TARGET_SHORT).*
ifeq ($(DESTDIR),)
	/sbin/ldconfig
endif
