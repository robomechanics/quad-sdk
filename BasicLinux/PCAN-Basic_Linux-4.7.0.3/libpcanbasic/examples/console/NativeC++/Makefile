# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - Build C++ examples 1 to 9 for PCAN-Basic
#
# Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
#
# Contact:     <linux@peak-system.com>
# Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
# Author:      Romain Tissier <r.tissier@peak-system.com>
#
BIN_OUTPUT = ..
TITLE := PCANBasic C++ examples

define do-make
@make -C 01_LookUpChannel BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 02_GetSetParameter BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 03_ManualRead BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 04_ManualWrite BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 05_TimerRead BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 06_TimerWrite BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 07_ThreadRead BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 08_EventDrivenRead BIN_OUTPUT=$(BIN_OUTPUT) $1
@make -C 09_TraceFiles BIN_OUTPUT=$(BIN_OUTPUT) $1
endef

define make-all
$(call do-make, all)
endef

define make-clean
$(call do-make, clean)
endef

all:
	@echo
	@echo "***"
	@echo "*** Making $(TITLE)..."
	@echo "***"
	$(make-all)

clean:
	@echo
	@echo "***"
	@echo "*** Cleaning $(TITLE)..."
	@echo "***"
	$(make-clean)

xeno rtai:
	$(call do-make, $@)
	
# Ignored targets
install uninstall: ;
