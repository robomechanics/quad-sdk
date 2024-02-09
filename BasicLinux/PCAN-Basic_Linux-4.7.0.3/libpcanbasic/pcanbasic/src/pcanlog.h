/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcanlog.h
 * @brief Function prototypes to log stuff
 * $Id: pcanlog.h 13814 2022-01-18 11:16:55Z Fabrice $
 *
 * Copyright (C) 2001-2022  PEAK System-Technik GmbH <www.peak-system.com>
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
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */

#ifndef __PCANLOG_H__
#define __PCANLOG_H__

/**
 * Defines log verbosity
 */
typedef enum {
	LVL_QUIET,		/**< log seen when using silent mode */
	LVL_NORMAL,		/**< default log */
	LVL_VERBOSE,	/**< log seen when using verbose mode */
	LVL_DEBUG,		/**< log seen when using debug mode */
	LVL_ALWAYS		/**< log always displayed */
} PCANLOG_LEVEL;


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn void pcanlog_set(PCANLOG_LEVEL lvl, char *filename, int showtime)
 * @brief Configures the logging system.
 *
 * @param[in] lvl The maximum level to be displayed
 * @param[in] filename The filename to write the log
 * @param[in] showtime State to prefix the log with a timestamp
 */
void pcanlog_set(const PCANLOG_LEVEL lvl, const char *filename, const int showtime);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Logs an entry (with a timestamp if optien is set)
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_log(const PCANLOG_LEVEL lvl, const char *fmt, ...);

/**
 * @fn void pcanlog_log(PCANLOG_LEVEL lvl, char *fmt, ...)
 * @brief Writes a raw message in the log
 *
 * @param[in] lvl level of the log
 * @param[in] fmt Formatted string
 */
void pcanlog_write(const PCANLOG_LEVEL lvl, const char *fmt, ...);

#ifdef __cplusplus
};
#endif

#endif /* __PCANLOG_H__ */
