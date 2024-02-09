/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcaninfo.c
 * @brief Tools to get information on PCAN devices
 * $Id: pcaninfo.c 14983 2022-05-12 14:15:41Z Fabrice $
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
 * Contact:    <linux@peak-system.com>
 * Maintainer: Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */
#include "pcaninfo.h"

/*
 * INCLUDES
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>		/* scandir */
#include <math.h>		/* floor */
#include <pcan.h>		/* PCAN HW types */
#include <pcanfd.h>		/* PCAN_ERROR_BUS types */
#include <time.h>		/* time */

#include "pcanlog.h"

#ifndef HW_PCI_FD
#define HW_PCI_FD 19
#endif
/*
 * DEFINES
 */
/** Sysfs path to retrieve pcan mmodule information */
#define PCAN_MODULE_PATH	"/sys/module/pcan"
/** Sysfs path to retrieve pcan-pci/pcmcia information */
#define PCAN_CLASS_PATH		"/sys/class/pcan"
/** prefixed used by pcan usb devices */
#define PCAN_USBMISC_PREFIX	"pcan"
/** Sysfs path to retrievedriver's clock_reference */
#define PCAN_CLK_REF_PATH	PCAN_CLASS_PATH "/clk_ref"
/** Sysfs path to retrieve version information */
#define PCAN_VERSION_PATH	PCAN_CLASS_PATH "/version"
/** @deprecated Path to retrieve version information */
#define PCAN_PROC_PATH		"/proc/pcan"

/**
 * @defgroup PCAN_FILEINFO PCAN files in found Sysfs
 *
 * @{
 */
/** @cond Doxygen_Suppress */
#define PCAN_FILEINFO_ADAPTER_NAME		"adapter_name"
#define PCAN_FILEINFO_ADAPTER_NB		"adapter_number"
#define PCAN_FILEINFO_ADAPTER_PARTNUM	"adapter_partnum"
#define PCAN_FILEINFO_ADAPTER_VERSION	"adapter_version"
#define PCAN_FILEINFO_BASE				"base"
#define PCAN_FILEINFO_NOM_BITRATE 		"nom_bitrate"
#define PCAN_FILEINFO_NOM_BRP 			"nom_brp"
#define PCAN_FILEINFO_NOM_SAMPLE_POINT	"nom_sample_point"
#define PCAN_FILEINFO_NOM_SJW 			"nom_sjw"
#define PCAN_FILEINFO_NOM_TSEG1			"nom_tseg1"
#define PCAN_FILEINFO_NOM_TSEG2			"nom_tseg2"
#define PCAN_FILEINFO_NOM_TQ			"nom_tq"
#define PCAN_FILEINFO_BTR0BTR1			"btr0btr1"
#define PCAN_FILEINFO_BUSLOAD			"bus_load"
#define PCAN_FILEINFO_BUSSTATE			"bus_state"
#define PCAN_FILEINFO_CLOCK				"clock"
#define PCAN_FILEINFO_CLK_DRIFT			"clk_drift"
#define PCAN_FILEINFO_CTRLNB 			"ctrlr_number"
#define PCAN_FILEINFO_DATA_BITRATE 		"data_bitrate"
#define PCAN_FILEINFO_DATA_BRP 			"data_brp"
#define PCAN_FILEINFO_DATA_SAMPLE_POINT	"data_sample_point"
#define PCAN_FILEINFO_DATA_SJW 			"data_sjw"
#define PCAN_FILEINFO_DATA_TSEG1		"data_tseg1"
#define PCAN_FILEINFO_DATA_TSEG2		"data_tseg2"
#define PCAN_FILEINFO_DATA_TQ			"data_tq"
#define PCAN_FILEINFO_DEV 				"dev"
#define PCAN_FILEINFO_DEV_NAME			"dev_name"
#define PCAN_FILEINFO_DEVICE 			"device"
#define PCAN_FILEINFO_DEVID 			"devid"
#define PCAN_FILEINFO_ERRORS 			"errors"
#define PCAN_FILEINFO_HWTYPE 			"hwtype"
#define PCAN_FILEINFO_INIT_FLAGS		"init_flags"
#define PCAN_FILEINFO_IRQ				"irq"
#define PCAN_FILEINFO_IRQS		 		"irqs"
#define PCAN_FILEINFO_MASS_STORAGE_MODE	"mass_storage_mode"
#define PCAN_FILEINFO_MINOR		 		"minor"
#define PCAN_FILEINFO_POWER 			"power"
#define PCAN_FILEINFO_READ 				"read"
#define PCAN_FILEINFO_RXERR				"rx_error_counter"
#define PCAN_FILEINFO_SN	 			"serial_number"
#define PCAN_FILEINFO_STATUS 			"status"
#define PCAN_FILEINFO_SUBSYSTEM			"subsystem"
#define PCAN_FILEINFO_TXERR				"tx_error_counter"
#define PCAN_FILEINFO_TYPE	 			"type"
#define PCAN_FILEINFO_UEVENT 			"uevent"
#define PCAN_FILEINFO_WRITE 			"write"
#define PCAN_FILEINFO_RX_FIFO_RATIO		"rx_fifo_ratio"
#define PCAN_FILEINFO_TX_FIFO_RATIO		"tx_fifo_ratio"
#define PCAN_FILEINFO_TS_FIXED			"ts_fixed"
/** @endcond */
/** @} */

/** legacy: sysfs pcan file prefix, used with pcan driver prior to v8.0 */
#define PCAN_FILEINFO_PREFIX_LEGACY	"pcan_"
/**
 * @def LEGACY_GET_FILEINFO_NAME(file)
 * @brief A macro that returns the legacy name of pcan files in PCAN_CLASS_PATH
 * */
#define LEGACY_GET_FILEINFO_NAME(file) (PCAN_FILEINFO_PREFIX_LEGACY file)

/**
 * @def MAX(x,y)
 * @brief A macro that returns the maximum of @a x and @a y.
 */
#define MAX(x,y) ((x) > (y) ? (x) : (y))
/**
 * @def MIN(x,y)
 * @brief A macro that returns the minimum of @a x and @a y.
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))

#define MAX_LENGTH_VERSION_STRING 256

/** "No error" status error code */
#define PCANINFO_ERR_OK 	0

/* PRIVATE FUNCTIONS DECLARATIONS */
/** function used by scandir to get all files except '.' and '..' */
static int classdir_selector(const struct dirent *ent);
static int classfile_selector(const struct dirent *ent);

/** function used to order "struct pcaninfo" */
static int compare_pcaninfo(const void*, const void*);

 /**
  * @fn int load_devinfo(struct pcaninfo * pci)
  * @brief Retrieves information for a PCAN device.
  *
  * @param pci [in,out] {buffer to store PCAN data.
  * 	classpath and name must be set.}
  * @return a PCANINFO_ERR_xx status code
  */
static int load_devinfo(struct pcaninfo * pci);
/**
 * @fn int parse_file(struct pcaninfo *pci, char *path, char *filename)
 * @brief Parses a sysfs pcan file and populates the struct pcaninfo buffer
 *
 * @param[in, out] pci buffer to store data
 * @param[in] path Path of the file to read
 * @param[in] filename Name of the file to read
 * @return a PCANINFO_ERR_xx status code
 */
static int parse_file(struct pcaninfo *pci, char *path, char *filename);

/**
 * @fn char * pretty_unit(unsigned long val, char * buffer, size_t len)
 * @brief Formats a value with a valid SI unit.
 *
 * @param[in] val Value to format
 * @param[in, out] Buffer to store the formatted string
 * @param[in] Size of the buffer
 * @return The 'buffer' param
 */
static char * pretty_unit(unsigned long val, char * buffer, size_t len);
/**
 * @fn char * pretty_bus_state(unsigned int state, char * buffer, size_t len)
 * @brief Formats a PCAN bus state.
 *
 * @param[in] val Value to format
 * @param[in, out] Buffer to store the formatted string
 * @param[in] Size of the buffer
 * @return The 'buffer' param
 */
static char * pretty_bus_state(unsigned int state, char * buffer, size_t len);

/* PRIVATE FUNCTIONS */
int classdir_selector(const struct dirent *ent) {
	if (ent->d_name[0] == '.') {
		if (ent->d_name[1] == 0)
			return 0;
		if (ent->d_name[1] == '.' &&
			ent->d_name[2] == 0)
				return 0;
	}
	if (ent->d_type == DT_REG)
		return 0;
	return 1;
}
int classfile_selector(const struct dirent *ent) {
	if (ent->d_name[0] == '.') {
		if (ent->d_name[1] == 0)
			return 0;
		if (ent->d_name[1] == '.' &&
			ent->d_name[2] == 0)
				return 0;
	}
	if (ent->d_type == DT_DIR)
		return 0;
	return 1;
}

static int compare_pcaninfo(const void* a, const void* b) {
	return ((const struct pcaninfo *)a)->minor - ((const struct pcaninfo *)b)->minor;
}

int load_devinfo(struct pcaninfo * pci) {
	struct dirent **ent;
	int i, n;
	char * path;
	FILE * f;

	/* read driver's reference clock (driver 8.11+) */
	f = fopen(PCAN_CLK_REF_PATH, "r");
	if (f != NULL) {
		char * line = NULL;
		size_t len = 0;
		ssize_t read = 0;
		/* read first line */
		read = getline(&line, &len, f);
		if (read != -1) {
			pci->clk_ref = strtoul(line, NULL, 0);
		}
		if (line) {
			free(line);
			line = NULL;
		}
		if (f) {
			fclose(f);
			f = NULL;
		}
	}
	else {
		pcanlog_log(LVL_NORMAL, "NOTICE: failed to open file (errno=%d) '%s', using default clk_ref.\n", errno, PCAN_CLK_REF_PATH);
	}

	/* initialization */
	n = strnlen(pci->classpath, PCANINFO_MAX_CHAR_SIZE) + strnlen(pci->name, PCANINFO_MAX_CHAR_SIZE) + 2;	/* 2 = '/' + '\0' */
	path = (char *) malloc(n);
	if (path == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to allocate memory to scan directory '%s/%s'.\n", pci->classpath, pci->name);
		return errno = ENOMEM;
	}
	snprintf(path, n, "%s/%s", pci->classpath, pci->name);
	pcanlog_log(LVL_DEBUG, "Scanning directory '%s'...\n", path);
	/* scan sys dirs */
	ent = NULL;
	n = scandir(path, &ent, classfile_selector, alphasort);
	/* foreach file, load info */
	for (i = 0; i < n; i++) {
#ifdef _DIRENT_HAVE_D_TYPE
		if (ent[i]->d_type == DT_REG)	/* skip directories & links*/
#endif
		/* SGr Note: this file is not a pcan one and reading it might
		 * lead to segfault with Xenomai */
		if (strcmp(ent[i]->d_name, PCAN_FILEINFO_UEVENT))
			parse_file(pci, path, ent[i]->d_name);

		/* SGr Note: fill "path" AFTER having read /sysfs so that 
		 * "dev_name" can be used instead of "path" in case it has 
		 * been discovered... */

		/* RT support: sysfs var "dev_name" holds the correct string to
		 * open device */
		if (pci->availflag_ex & PCANINFO_FLAG_EX_DEV_NAME) {
			strncpy(pci->path, pci->dev_name, sizeof(pci->path));
		} else {
			/* retrocompatibility with older pcan drivers
			 * (RT not supported) */
			snprintf(pci->path, sizeof(pci->path), "/dev/%s", pci->name);
		}

		free(ent[i]);
	}
	switch (pci->hwtype) {
		case HW_DONGLE_PRO:
		case HW_DONGLE_SJA:
			pci->hwcategory = PCANINFO_HW_DNG;
			pci->hwindex = pci->minor - 79;
			break;
		case HW_DONGLE_PRO_EPP:
		case HW_DONGLE_SJA_EPP:
			pci->hwcategory = PCANINFO_HW_DNG;
			pci->hwindex = pci->minor - 87;
			break;
		case HW_ISA:
		case HW_ISA_SJA:
			pci->hwcategory = PCANINFO_HW_ISA;
			pci->hwindex = pci->minor - 71;
			break;
		case HW_PCI:
		case HW_PCI_FD:
			pci->hwcategory = PCANINFO_HW_PCI;
			pci->hwindex = pci->minor - 0;
			break;
		case HW_USB:
		case HW_USB_PRO:
		case HW_USB_PRO_FD:
		case HW_USB_FD:
		case HW_USB_X6:
			pci->hwcategory = PCANINFO_HW_USB;
			pci->hwindex = pci->minor - 31;
			break;
		case HW_PCCARD:
			pci->hwcategory = PCANINFO_HW_PCC;
			pci->hwindex = pci->minor - 63;
			break;
		default:
			pci->hwcategory = PCANINFO_HW_NONE;
			pci->hwindex = 0;
			break;
	}
	/* mark the structure as initialized */
	pci->availflag |= PCANINFO_FLAG_INITIALIZED;
	time(&pci->time_update);
	/* uninitialize */
	if (ent)
		free(ent);
	if (path)
		free(path);
	return PCANINFO_ERR_OK;
}

int parse_file(struct pcaninfo *pci, char *path, char *filename) {
	char * filepath;
	FILE * f;
	char * line;
	size_t len;
	ssize_t read;

	/* Invalid argument ? */
	if (pci == NULL || path == NULL || filename == NULL ||
			path[0] == 0 || filename[0] == 0)
		return errno = EINVAL;
	/* initialization */
	filepath = (char *) malloc(strlen(path) + strlen(filename) + 2);
	if (filepath == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to allocate memory to parse file '%s/%s'.\n", path, filename);
		return errno = ENOMEM;
	}
	sprintf(filepath, "%s/%s", path, filename);
	pcanlog_log(LVL_DEBUG, "Parsing file '%s'...\n", filepath);

	line = NULL;
	len = 0;
	/* open file */
	f = fopen(filepath, "r");
	if (f == NULL) {
		errno = ENOENT;
		pcanlog_log(LVL_NORMAL, "ERROR: failed to open file '%s'.\n", filepath);
	}
	else
	{
		/* read first line */
		read = getline(&line, &len, f);
		if (read != -1) {
			/* fill in the right PCANINFO parameter based on filename */
			if (strcmp(filename, PCAN_FILEINFO_ADAPTER_NAME) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->adapter_name, line, len);
				if (len >= 1 && pci->adapter_name[len - 1] == '\n')
					pci->adapter_name[len - 1] = 0;
				pci->adapter_name[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_ADAPTER_NAME;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ADAPTER_NB) == 0) {
				pci->adapter_nb = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_ADAPTER_NB;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ADAPTER_VERSION) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->adapter_version, line, len);
				if (len >= 1 && pci->adapter_version[len - 1] == '\n')
					pci->adapter_version[len - 1] = 0;
				pci->adapter_version[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_ADAPTER_VERSION;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ADAPTER_PARTNUM) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->adapter_partnum, line, len);
				if (len >= 1 && pci->adapter_partnum[len - 1] == '\n')
					pci->adapter_partnum[len - 1] = 0;
				pci->adapter_partnum[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag_ex |= PCANINFO_FLAG_EX_ADAPTER_PARTNUM;
			} 
			else if (strcmp(filename, PCAN_FILEINFO_NOM_BITRATE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_NOM_BITRATE)) == 0) {
				pci->nom_bitrate = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_NOM_BITRATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BTR0BTR1) == 0) {
				pci->btr0btr1 = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BTR0BTR1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_CLOCK) == 0 ||
				strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_CLOCK)) == 0) {
				pci->clock = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_CLOCK;
			}
			else if (strcmp(filename, PCAN_FILEINFO_CLK_DRIFT) == 0) {
				pci->clk_drift = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_CLK_DRIFT;
			}
			else if (strcmp(filename, PCAN_FILEINFO_CTRLNB) == 0) {
				pci->ctrlnb = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_CTRLNB;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_BITRATE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DATA_BITRATE)) == 0) {
				pci->data_bitrate = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_DATA_BITRATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DEV) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DEV)) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->dev, line, len);
				if (len >= 1 && pci->dev[len - 1] == '\n')
					pci->dev[len - 1] = 0;
				pci->dev[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_DEV;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DEV_NAME) == 0) {
				len = MIN(sizeof(pci->dev_name), read);
				strncpy(pci->dev_name, line, len);
				if (len >= 1 && pci->dev_name[len - 1] == '\n')
					pci->dev_name[len - 1] = 0;
				pci->dev_name[sizeof(pci->dev_name) - 1] = 0;
				pci->availflag_ex |= PCANINFO_FLAG_EX_DEV_NAME;

				/* SGr Note: this field, if exists, must be used
				 * as device path, instead of "path", which is
				 * compatible only with the non-RT version of
				 * the pcan driver. */
			}
			else if (strcmp(filename, PCAN_FILEINFO_DEVID) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_DEVID)) == 0) {
				pci->devid = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_DEVID;
			}
			else if (strcmp(filename, PCAN_FILEINFO_ERRORS) == 0) {
				pci->errors = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_ERRORS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_HWTYPE) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_HWTYPE)) == 0) {
				pci->hwtype = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_HWTYPE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_INIT_FLAGS) == 0) {
				pci->init_flags = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_INIT_FLAGS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_IRQS) == 0) {
				pci->irqs = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_IRQS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_MASS_STORAGE_MODE) == 0) {
				pci->mass_storage_mode = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_MASS_STORAGE_MODE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_MINOR) == 0 ||
					strcmp(filename, LEGACY_GET_FILEINFO_NAME(PCAN_FILEINFO_MINOR)) == 0) {
				pci->minor = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_MINOR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_READ) == 0) {
				pci->read = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_READ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_SN) == 0) {
				pci->sn = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_SN;
			}
			else if (strcmp(filename, PCAN_FILEINFO_STATUS) == 0) {
				pci->status = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_STATUS;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TYPE) == 0) {
				len = MIN(PCANINFO_MAX_CHAR_SIZE, read);
				strncpy(pci->type, line, len);
				if (len >= 1 && pci->type[len - 1] == '\n')
					pci->type[len - 1] = 0;
				pci->type[PCANINFO_MAX_CHAR_SIZE - 1] = 0;
				pci->availflag |= PCANINFO_FLAG_TYPE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_WRITE) == 0) {
				pci->write = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_WRITE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BASE) == 0) {
				pci->base = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BASE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_IRQ) == 0) {
				pci->irq = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_IRQ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BUSLOAD) == 0) {
				pci->bus_load = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BUSLOAD;
			}
			else if (strcmp(filename, PCAN_FILEINFO_BUSSTATE) == 0) {
				pci->bus_state = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_BUSSTATE;
			}
			else if (strcmp(filename, PCAN_FILEINFO_RXERR) == 0) {
				pci->rxerr = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_RXERR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TXERR) == 0) {
				pci->txerr = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_TXERR;
			}
			else if (strcmp(filename, PCAN_FILEINFO_RX_FIFO_RATIO) == 0) {
				pci->rx_fifo_ratio = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_RX_FIFO_RATIO;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TX_FIFO_RATIO) == 0) {
				pci->tx_fifo_ratio = strtoul(line, NULL, 0);
				pci->availflag |= PCANINFO_FLAG_TX_FIFO_RATIO;
			}
			/* handle extra flags with availflag_ex */
			else if (strcmp(filename, PCAN_FILEINFO_NOM_BRP) == 0) {
				pci->nom_brp = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_BRP;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_SAMPLE_POINT) == 0) {
				pci->nom_sample_point = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_SAMPLE_POINT;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_SJW) == 0) {
				pci->nom_sjw = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_SJW;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_TSEG1) == 0) {
				pci->nom_tseg1 = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_TSEG1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_TSEG2) == 0) {
				pci->nom_tseg2 = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_TSEG2;
			}
			else if (strcmp(filename, PCAN_FILEINFO_NOM_TQ) == 0) {
				pci->nom_tq = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_NOM_TQ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_BRP) == 0) {
				pci->data_brp = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_BRP;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_SAMPLE_POINT) == 0) {
				pci->data_sample_point = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_SAMPLE_POINT;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_SJW) == 0) {
				pci->data_sjw = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_SJW;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_TSEG1) == 0) {
				pci->data_tseg1 = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_TSEG1;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_TSEG2) == 0) {
				pci->data_tseg2 = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_TSEG2;
			}
			else if (strcmp(filename, PCAN_FILEINFO_DATA_TQ) == 0) {
				pci->data_tq = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_DATA_TQ;
			}
			else if (strcmp(filename, PCAN_FILEINFO_TS_FIXED) == 0) {
				pci->ts_fixed = strtoul(line, NULL, 0);
				pci->availflag_ex |= PCANINFO_FLAG_EX_TS_FIXED;
			}
			else {
				/* unsupported files */
				if (strcmp(filename, PCAN_FILEINFO_UEVENT) != 0) {
					pcanlog_log(LVL_DEBUG, "WARNING: unsupported file '%s' (%s).\n", filename, path);
				}
			}
		}
		else {
			pcanlog_log(LVL_NORMAL, "ERROR: failed to read line in file (%s).\n", filename, path);
			errno = EIO;
		}
	}
	/* uninitialize */
	if (line)
		free(line);
	if (f)
		fclose(f);
	if (filepath)
		free(filepath);
	return errno;
}

char * pretty_unit(unsigned long val, char * buffer, size_t len) {
	float fval;
	unsigned long e;
	char unit;

	/* identify SI prefix */
	if (val >= 1000000) {
		fval = val / 1000000.0f;
		unit = 'M';
	}
	else if (val >= 1000) {
		fval = val / 1000.0f;
		unit = 'k';
	}
	else {
		fval = floor(val);
		unit = '\0';
	}
	/* format buffer avoiding useless decimals */
	e = floor(fval);
	if (fval - e > 0)
		snprintf(buffer, len, "%.03f %c", fval, unit);
	else
		snprintf(buffer, len, "%lu %c", e, unit);
	return buffer;
}

char * pretty_bus_state(unsigned int state, char * buffer, size_t len) {

	switch (state) {
	case PCANFD_ERROR_PASSIVE:
		snprintf(buffer, len, "Passive");
		break;
	case PCANFD_ERROR_WARNING:
		snprintf(buffer, len, "Warning");
		break;
	case PCANFD_ERROR_BUSOFF:
		snprintf(buffer, len, "BUS OFF");
		break;
	case PCANFD_ERROR_ACTIVE:
		snprintf(buffer, len, "OK");
		break;
	default:
		snprintf(buffer, len, "Closed / Unknown");
		break;
	}
	return buffer;
}

/*
 * GLOBAL FUNCTIONS
 */

int pcaninfo_update(struct pcaninfo * pci) {
	if (pci == NULL || pci->classpath == NULL || pci->name[0] == 0)
		return errno = EINVAL;
	return load_devinfo(pci);
}

int pcaninfo_get(struct pcaninfo_list ** pcilist, int do_init) {
	struct pcaninfo_list *pcil;
	struct dirent **entpcan;
	int npcan, i, len;
	int ires;
	char * path;

	/* scan sys 'pcan' dir */
	ires = 0;
	entpcan = NULL;
	path = PCAN_CLASS_PATH;
	npcan = scandir(path, &entpcan, classdir_selector, alphasort);
	if (npcan < 0) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to scan directory (errno=%d) '%s'\n", errno, path);
		npcan = 0;
	}
	pcanlog_log(LVL_VERBOSE, "Found %d devices in '%s'\n", npcan, path);
	/* initialize each device information */
	len = MAX(0, npcan);
	i = sizeof(*pcil) + len * sizeof(pcil->infos[0]);
	pcil = (struct pcaninfo_list *) calloc(1, i);
	if (pcil == NULL) {
		ires = ENOMEM;
		goto pcaninfo_get_free;
	}
	pcil->length = len;
	if (npcan > 0) {
		for (i = 0; i < npcan; i++) {
			pcil->infos[i].classpath = PCAN_CLASS_PATH;
			strncpy(pcil->infos[i].name, entpcan[i]->d_name, entpcan[i]->d_reclen);
			if (do_init)
				load_devinfo(&pcil->infos[i]);
			free(entpcan[i]);
		}
		/* re-order pcil by minor:
		 * 	if more than 9 interfaces with the same type are used,
		 *	the order of the interfaces is mixed:
		 *	pcanpcifd0, pcanpcifd1, pcanpcifd10, pcanpcifd11, etc.
		 */
		qsort(pcil->infos, pcil->length, sizeof(pcil->infos[0]), &compare_pcaninfo);
	}

	/* read driver version */
	pcaninfo_driver_version(pcil->version, sizeof(pcil->version));

	*pcilist = pcil;
	/* uninitialize */
pcaninfo_get_free:
	if (entpcan)
		free(entpcan);
	return ires;
}

void pcaninfo_output(struct pcaninfo * pci) {
	char tmp[PCANINFO_MAX_CHAR_SIZE];
	int separator;
	/* Invalid argument ? */
	if (pci == NULL)
		return;

	fprintf(stdout, "  * %s: (%s/%s)\n", pci->name, pci->classpath, pci->name);
	fprintf(stdout, "  \t- file: %s\n", pci->path);
	//fprintf(stdout, "  \t- file: %s\n", pci->dev_name);

	/* print device info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_DEV) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_DEV, pci->dev);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DEV_NAME) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_DEV_NAME, pci->dev_name);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_MINOR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_MINOR, pci->minor);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BASE) {
		fprintf(stdout, "  \t- %s: 0x%X\n", PCAN_FILEINFO_BASE, pci->base);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_IRQ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_IRQ, pci->irq);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print hardware info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_NAME) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_ADAPTER_NAME, pci->adapter_name);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_ADAPTER_PARTNUM) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_ADAPTER_PARTNUM, pci->adapter_partnum);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_NB) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_ADAPTER_NB, pci->adapter_nb);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_VERSION) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_ADAPTER_VERSION, pci->adapter_version);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_TYPE) {
		fprintf(stdout, "  \t- %s: \"%s\"\n", PCAN_FILEINFO_TYPE, pci->type);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_HWTYPE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_HWTYPE, pci->hwtype);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_DEVID) {
		fprintf(stdout, "  \t- %s: 0x%02X\n", PCAN_FILEINFO_DEVID, pci->devid);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_SN) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_SN, pci->sn);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_CTRLNB) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_CTRLNB, pci->ctrlnb);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_MASS_STORAGE_MODE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_MASS_STORAGE_MODE, pci->mass_storage_mode);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Channel connection info */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_CLOCK) {
		fprintf(stdout, "  \t- %s: %sHz\n", PCAN_FILEINFO_CLOCK,
				pretty_unit(pci->clock, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_NOM_BITRATE) {
		fprintf(stdout, "  \t- %s: %sBit/s\n", PCAN_FILEINFO_NOM_BITRATE,
				pretty_unit(pci->nom_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BTR0BTR1) {
		fprintf(stdout, "  \t- %s: 0x%X\n", PCAN_FILEINFO_BTR0BTR1,
				pci->btr0btr1);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_DATA_BITRATE) {
		fprintf(stdout, "  \t- %s: %sBit/s\n", PCAN_FILEINFO_DATA_BITRATE,
			pretty_unit(pci->data_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_INIT_FLAGS) {
		fprintf(stdout, "  \t- %s: 0x%X\n", PCAN_FILEINFO_INIT_FLAGS, pci->init_flags);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_CLK_DRIFT) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_CLK_DRIFT, pci->clk_drift);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_TS_FIXED) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_TS_FIXED, pci->ts_fixed);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Channel More connection info */
	separator = 0;
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_BRP) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_BRP, pci->nom_brp);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_SAMPLE_POINT) {
		fprintf(stdout, "  \t- %s: %.02f%%\n", PCAN_FILEINFO_NOM_SAMPLE_POINT, pci->nom_sample_point/100.0f);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_SJW) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_SJW, pci->nom_sjw);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG1) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_TSEG1, pci->nom_tseg1);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG2) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_TSEG2, pci->nom_tseg2);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TQ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_NOM_TQ, pci->nom_tq);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_BRP) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_BRP, pci->data_brp);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_SAMPLE_POINT) {
		fprintf(stdout, "  \t- %s: %.02f%%\n", PCAN_FILEINFO_DATA_SAMPLE_POINT, pci->data_sample_point/100.0f);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_SJW) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_SJW, pci->data_sjw);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG1) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_TSEG1, pci->data_tseg1);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG2) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_TSEG2, pci->data_tseg2);
		separator++;
	}
	if (pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TQ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_DATA_TQ, pci->data_tq);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print Bus stats */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_BUSSTATE) {
		fprintf(stdout, "  \t- %s: %s (%d)\n", PCAN_FILEINFO_BUSSTATE,
				pretty_bus_state(pci->bus_state, tmp, PCANINFO_MAX_CHAR_SIZE),
				pci->bus_state);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_BUSLOAD) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_BUSLOAD,
				pci->bus_load);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_RXERR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_RXERR, pci->rxerr);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_TXERR) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_TXERR, pci->txerr);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_RX_FIFO_RATIO) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_RX_FIFO_RATIO, pci->rx_fifo_ratio);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_TX_FIFO_RATIO) {
		fprintf(stdout, "  \t- %s: %d%%\n", PCAN_FILEINFO_TX_FIFO_RATIO, pci->tx_fifo_ratio);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");

	/* print IO stats */
	separator = 0;
	if (pci->availflag & PCANINFO_FLAG_IRQS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_IRQS, pci->irqs);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_STATUS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_STATUS, pci->status);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_ERRORS) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_ERRORS, pci->errors);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_READ) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_READ, pci->read);
		separator++;
	}
	if (pci->availflag & PCANINFO_FLAG_WRITE) {
		fprintf(stdout, "  \t- %s: %d\n", PCAN_FILEINFO_WRITE, pci->write);
		separator++;
	}
	if (separator)
		fprintf(stdout, "  \t-----------------\n");
}

void pcaninfo_output_summary(struct pcaninfo * pci, char* handle_info) {
	char tmp[PCANINFO_MAX_CHAR_SIZE];
	/* Invalid argument ? */
	if (pci == NULL)
		return;

	fprintf(stdout, "  * %s:", pci->name);
	/* display TPCANHandle info if any */
	if (handle_info != NULL) {
		fprintf(stdout, " %s", handle_info);
	}
	/* display bitrate if bus is initialized */
	if (pci->bus_state != 0)
	{
		if (pci->availflag & PCANINFO_FLAG_NOM_BITRATE) {
			fprintf(stdout, " @ %sBit/s",
					pretty_unit(pci->nom_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
			/* append data bitrate if bus is initialized as CAN FD */
			if ((pci->init_flags & PCANFD_INIT_FD) == PCANFD_INIT_FD) {
				if (pci->availflag & PCANINFO_FLAG_DATA_BITRATE) {
					fprintf(stdout, " | %sBit/s",
							pretty_unit(pci->data_bitrate, tmp, PCANINFO_MAX_CHAR_SIZE));
				}
			}
		}
	}
	/* display adapter name */
	if (pci->availflag & PCANINFO_FLAG_ADAPTER_NAME) {
		fprintf(stdout, ", %s", pci->adapter_name);
		/* display controller nb */
		if (pci->availflag & PCANINFO_FLAG_CTRLNB) {
			fprintf(stdout, " #%d", 1 + pci->ctrlnb);
		}
	}
	/* display device id */
	if (pci->availflag & PCANINFO_FLAG_DEVID) {
		fprintf(stdout, ", devid=0x%02X", pci->devid);
	}
	/* display device path */
	fprintf(stdout, " (%s/%s)", pci->classpath, pci->name);
	fprintf(stdout, "\n");
}


int pcaninfo_print(void) {
	struct pcaninfo_list *pcilist;
	int i, ires;

	pcilist = NULL;
	ires = pcaninfo_get(&pcilist, 1);
	if (ires != 0)
		return ires;

	if (pcilist->version[0] != 0)
		fprintf(stdout, "PCAN driver version: %s\n\n", pcilist->version);
	else
		fprintf(stdout, "PCAN driver not found\n\n");
	fprintf(stdout, "Found %d PCAN devices\n", pcilist->length);
	for (i = 0; i < pcilist->length; i++) {
		pcaninfo_output(&pcilist->infos[i]);
		fprintf(stdout, "\n");
	}
	free(pcilist);
	return ires;
}

int pcaninfo_driver_version(char *buffer, unsigned int size) {
	FILE * f;
	char * line;
	size_t len, lenver;
	ssize_t read;

	/* Invalid argument ? */
	if (buffer == NULL || size == 0)
		return errno = EINVAL;
	line = NULL;
	len = 0;
	/* open file */
	f = fopen(PCAN_VERSION_PATH, "r");
	if (f == NULL) {
		pcanlog_log(LVL_NORMAL, "ERROR: failed to open file (errno=%d) '%s'.\n", errno, PCAN_VERSION_PATH);
		errno = ENOENT;
		/* try to detect old PCAN driver */
		f = fopen(PCAN_PROC_PATH, "r");
		if (f != NULL)
			snprintf(buffer, size, "prior to 8.0");

	}
	else
	{
		/* read first line */
		read = getline(&line, &len, f);
		if (read != -1) {
			lenver = MIN(size, read);
			strncpy(buffer, line, lenver);
			if (lenver > 1 && buffer[lenver - 1] == '\n')
				buffer[lenver - 1] = 0;
		}
	}
	/* uninitialize */
	if (line)
		free(line);
	if (f)
		fclose(f);
	return 0;
}

char* pcaninfo_bitrate_to_string(struct pcaninfo * pci, char *buffer, unsigned int size) {
	char tmpValue[PCANINFO_MAX_CHAR_SIZE];
	char tmpValueUnit[PCANINFO_MAX_CHAR_SIZE];
	
	memset(buffer, 0, size);
	if ((pci->availflag & PCANINFO_FLAG_NOM_BITRATE) && pci->nom_bitrate > 0) {
		snprintf(tmpValueUnit, sizeof(tmpValueUnit), "Nominal: %sBit/s", pretty_unit(pci->nom_bitrate, tmpValue, PCANINFO_MAX_CHAR_SIZE));
		strcat(buffer, tmpValueUnit);
	}
	if ((pci->availflag & PCANINFO_FLAG_BTR0BTR1) && pci->btr0btr1 > 0) {
		snprintf(tmpValueUnit, sizeof(tmpValueUnit), " (0x%X)", pci->btr0btr1);
		strcat(buffer, tmpValueUnit);
	}
	if ((pci->availflag & PCANINFO_FLAG_DATA_BITRATE) && pci->data_bitrate > 0) {
		snprintf(tmpValueUnit, sizeof(tmpValueUnit), ", Data: %sBit/s", pretty_unit(pci->data_bitrate, tmpValue, PCANINFO_MAX_CHAR_SIZE));
		strcat(buffer, tmpValueUnit);
	}
	if (pci->availflag & PCANINFO_FLAG_CLOCK) {
		snprintf(tmpValueUnit, sizeof(tmpValueUnit), " (%sHz)", pretty_unit(pci->clock, tmpValue, PCANINFO_MAX_CHAR_SIZE));
		strcat(buffer, tmpValueUnit);
	}
	return buffer;
}

char* pcaninfo_bitrate_to_init_string(struct pcaninfo * pci, char *buffer, unsigned int size) {
	char tmpValue[PCANINFO_MAX_CHAR_SIZE];

	memset(buffer, 0, size);
	if ((pci->availflag & PCANINFO_FLAG_CLOCK) && pci->clock > 0) {
		snprintf(tmpValue, sizeof(tmpValue), "f_clock=%u,", pci->clock);
		strcat(buffer, tmpValue);
	}

	if ((pci->availflag_ex & PCANINFO_FLAG_EX_NOM_BRP)) {
		snprintf(tmpValue, sizeof(tmpValue), "nom_brp=%d,", pci->nom_brp);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG1)) {
		snprintf(tmpValue, sizeof(tmpValue), "nom_tseg1=%d,", pci->nom_tseg1);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_NOM_TSEG2)) {
		snprintf(tmpValue, sizeof(tmpValue), "nom_tseg2=%d,", pci->nom_tseg2);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_NOM_SJW)) {
		snprintf(tmpValue, sizeof(tmpValue), "nom_sjw=%d,", pci->nom_sjw);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_DATA_BRP)) {
		snprintf(tmpValue, sizeof(tmpValue), "data_brp=%d,", pci->data_brp);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG1)) {
		snprintf(tmpValue, sizeof(tmpValue), "data_tseg1=%d,", pci->data_tseg1);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_DATA_TSEG2)) {
		snprintf(tmpValue, sizeof(tmpValue), "data_tseg2=%d,", pci->data_tseg2);
		strcat(buffer, tmpValue);
	}
	if ((pci->availflag_ex & PCANINFO_FLAG_EX_DATA_SJW)) {
		snprintf(tmpValue, sizeof(tmpValue), "data_sjw=%d,", pci->data_sjw);
		strcat(buffer, tmpValue);
	}
	return buffer;
}

const char* pcaninfo_hw_to_string(enum pcaninfo_hw hw, int no_bus_trailing) {
	char * res;

	res = "";
	switch (hw) {
	case PCANINFO_HW_DNG:
		res = (no_bus_trailing) ? "PCAN_DNG" : "PCAN_DNGBUS";
		break;
	case PCANINFO_HW_ISA:
		res = (no_bus_trailing) ? "PCAN_ISA" : "PCAN_ISABUS";
		break;
	case PCANINFO_HW_LAN:
		res = (no_bus_trailing) ? "PCAN_LAN" : "PCAN_LANBUS";
		break;
	case PCANINFO_HW_PCC:
		res = (no_bus_trailing) ? "PCAN_PCC" : "PCAN_PCCBUS";
		break;
	case PCANINFO_HW_PCI:
		res = (no_bus_trailing) ? "PCAN_PCI" : "PCAN_PCIBUS";
		break;
	case PCANINFO_HW_PEAKCAN:
		res = (no_bus_trailing) ? "PCAN_CAN" : "PCAN_CANBUS";
		break;
	case PCANINFO_HW_USB:
		res = (no_bus_trailing) ? "PCAN_USB" : "PCAN_USBBUS";
		break;
	case PCANINFO_HW_VIRTUAL:
		res = (no_bus_trailing) ? "PCAN_VIRTUAL" : "PCAN_VIRTUALBUS";
		break;
	case PCANINFO_HW_NONE:
		res = (no_bus_trailing) ? "PCAN_NONE" : "PCAN_NONEBUS";
		break;
	default:
		res = "UNKNOWN";
		break;
	}
	return res;
}

int pcaninfo_parse_version(const char* version, struct pcaninfo_version* buffer) {
	int res = 0;
	char str[MAX_LENGTH_VERSION_STRING];
	int len;
	char* tok;
	char* saveptr1 = NULL;
	int* number;
	const char* sep = ".";

	if (buffer == NULL || version == NULL) {
		res = EINVAL;
		goto pcaninfo_parse_version_exit;
	}
	memset(buffer, 0, sizeof(*buffer));
	memset(str, 0, sizeof(str));

	len = strnlen(version, MAX_LENGTH_VERSION_STRING);
	strncpy(str, version, len);
	
	number = &buffer->major;
	/* version should be formatted as %MAJOR%.%MINOR%.%PATCH%.%BUILD% */
	tok = strtok_r(str, sep, &saveptr1);
	while (tok && number) {
		*number = atoi(tok);
		if (number == &buffer->major) {
			buffer->status |= PCB_VERSION_MAJOR_SET;
			number = &buffer->minor;
		}
		else if (number == &buffer->minor) {
			buffer->status |= PCB_VERSION_MINOR_SET;
			number = &buffer->patch;
		}
		else if (number == &buffer->patch) {
			buffer->status |= PCB_VERSION_PATCH_SET;
			number = &buffer->build;
		}
		else if (number == &buffer->build) {
			buffer->status |= PCB_VERSION_BUILD_SET;
			number = NULL;
		}
		else {
			number = NULL;
		}
		tok = strtok_r(NULL, sep, &saveptr1);
	}

pcaninfo_parse_version_exit:
	return res;
}