# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - global Makefile for all Linux components of libpcanbasic
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH
#
# Contact: <linux@peak-system.com>
# Author:  Stephane Grosjean <s.grosjean@peak-system.com>
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
#
define do-make
@echo
@ echo "***"
@ echo "*** Processing 'pcanbasic' directory..."
@ echo "***"
@make -C pcanbasic $1
@echo
@ echo "***"
@ echo "*** Processing 'pcaninfo' directory..."
@ echo "***"
@make -C pcaninfo $1
@echo
@ echo "***"
@ echo "*** Processing 'examples' directory..."
@ echo "***"
@make -C examples $1
endef

define make-all
$(call do-make, all)
endef

define make-clean
$(call do-make, clean)
endef

define make-install
$(call do-make, install)
endef

define make-uninstall
$(call do-make, uninstall)
endef

define make-xeno
$(call do-make, xeno)
endef

define make-rtai
$(call do-make, rtai)
endef

all:
	$(make-all)

clean:
	$(make-clean)

install:
	$(make-install)

uninstall:
	$(make-uninstall)

xeno:
	$(make-xeno)

rtai:
	$(make-rtai)
