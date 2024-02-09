/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * private.h
 *
 * the shared library to unify the interface to the PEAK-System CAN[FD] devices
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
 * Contact:       <linux@peak-system.com>
 * Author:        Stephane Grosjean <s.grosjean@peak-system.com>
 * Contributions: Klaus Hitschler <klaus.hitschler@gmx.de>
 */
#ifndef __private_h__
#define __private_h__

#if !defined(NO_RT) && !defined(__COBALT__)
#include <rtdm/rtdm.h>

#define __open(x, y)		rt_dev_open(x, y)
#define __errno_ioctl(x, y, z)	-rt_dev_ioctl(x, y, z)
#define __ioctl(x, y, z)	rt_dev_ioctl(x, y, z)
#define __close(fp)		rt_dev_close(fp)

#define __fprintf		fprintf

#else
static inline int __open(const char *pfn, int flg)
{
	int fd = open(pfn, flg);
	return (fd >= 0) ? fd : -errno;
}
#define __errno_ioctl(x, y, z)	(ioctl(x, y, z) ? errno : 0)
#define __ioctl(x, y, z)	ioctl(x, y, z)
#define __close(fp)		close(fp)

#define __fprintf		fprintf
#endif

#define LOCAL_STRING_LEN	64	/* length of internal used strings */

struct pcan_handle {
	char szVersionString[LOCAL_STRING_LEN];
	char szDevicePath[LOCAL_STRING_LEN];
	int  fd;
};

#endif /* __private_h__ */
