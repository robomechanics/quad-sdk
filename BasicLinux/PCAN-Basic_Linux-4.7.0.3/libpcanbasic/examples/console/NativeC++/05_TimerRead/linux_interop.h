/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * linux_interop.h - Override Windows functions
 *
 * Copyright (C) 2001-2020  PEAK System-Technik GmbH <www.peak-system.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:    <linux@peak-system.com>
 * Maintainer:  Fabrice Vergnaud <f.vergnaud@peak-system.com>
 * Author:      Romain Tissier <r.tissier@peak-system.com>
 */
#include <iostream>
#include <thread>
#define UINT32 uint32_t
#include <limits.h>
#define MAX_PATH PATH_MAX
#define sprintf_s snprintf
#include <cstring>
#define strcpy_s(destination, destination_size, source) strcpy(destination, source)

#ifndef _getch_stub
#define _getch_stub
#include <termios.h>
#include <unistd.h>
static int _getch() {
	struct termios config;
	tcgetattr(STDIN_FILENO, &config);
	struct termios saved_config = config;
	config.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &config);
	int res = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &saved_config);
	return res;
}
#endif
