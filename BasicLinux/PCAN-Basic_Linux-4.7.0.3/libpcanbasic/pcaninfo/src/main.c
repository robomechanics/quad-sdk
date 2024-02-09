/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file main.c
 * $Id:
 *
 * Merely handles all API entry points (from Windows) by
 * calling corresponding Linux functions.
 * PCANBasic Logging features are handled here.
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
 * PCAN is a registered Trademark of PEAK-System Germany GmbH
 *
 * Contact:      <linux@peak-system.com>
 * Maintainer:   Fabrice Vergnaud <f.vergnaud@peak-system.com>
 */

#include "pcaninfo.h"
#include "pcanlog.h"
#include "pcbcore.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <dirent.h>
#include <dlfcn.h>

#include "version.h"

#define BUF_SIZE	100
/* Definitions to get PCAN-Basic library's version dynamically */
#define PCANBASIC_LIB_FILE "libpcanbasic.so"
#define PCANBASIC_GETVALUE "CAN_GetValue"

static const char *exec_name;

/* Definition of command-line options. */
static void print_info(void);
static int print_usage(int error);
static void print_help(void);
static void print_version(void);
static void print_version_pcanbasic(void);

/* Flag set by '--verbose'. */
static int verbose_flag;
/* Flag set by '--debug'. */
static int debug_flag;
/* Flag set by '--all'. */
static int resume_flag;

static struct option long_options[] = {
	/* These options set a flag. */
	{ "debug", no_argument, &debug_flag, 1 },
	{ "help", no_argument, 0, 'h' },
	{ "verbose", no_argument, &verbose_flag, 1 },
	{ "all", no_argument, &resume_flag, 0 },
	{ "list", no_argument, &resume_flag, 1 },
	{ 0, 0, 0, 0 }
};

char * pretty_tpcanhandle(TPCANHandle channel, char * buf, int size) {
	enum pcaninfo_hw hw = PCANINFO_HW_NONE;
	int index = 0;
	/* get the device's hardware category and minor/index from the channel */
	if (channel > 0xFF) {
		hw = ((channel & 0xFF00) >> 8);
		index = (channel & 0xFF);
	}
	else {
		hw = ((channel & 0xF0) >> 4);
		index = (channel & 0x0F);
	}
	switch(hw) {
	case PCANINFO_HW_DNG:
	case PCANINFO_HW_ISA:
	case PCANINFO_HW_LAN:
	case PCANINFO_HW_PCC:
	case PCANINFO_HW_PCI:
	case PCANINFO_HW_PEAKCAN:
	case PCANINFO_HW_USB:
	case PCANINFO_HW_VIRTUAL:
		snprintf(buf, size, "%s%d", pcaninfo_hw_to_string(hw, 0), index);
		break;
	case PCANINFO_HW_NONE:
	default:
		snprintf(buf, size, "%s", pcaninfo_hw_to_string(hw, 0));
		break;
	}
	return buf;
}

void print_info(void) {
	fprintf(stdout, "'pcaninfo' lists all known PCAN devices and outputs information for each one.\n");

}

int print_usage(int error) {
	return fprintf(error ? stderr : stdout, "Usage: %s [OPTION] [device_name_1] [device_name_2] [...]\n", exec_name);
}

void print_help(void) {
	fprintf(stdout, "  -h, --help        show this help\n");
	fprintf(stdout, "  -g, --debug       display debug messages\n");
	fprintf(stdout, "  -a, --all         display a detailed list of PCAN-Basic handles\n");
	fprintf(stdout, "  -l, --list        display a short list of PCAN-Basic handles (default)\n");
	fprintf(stdout, "  -v, --verbose     display more messages\n");
}

void print_version(void) {
	fprintf(stdout, ("%s version: %d.%d.%d.%d\n\n"), exec_name, VERSION_MAJOR,
		VERSION_MINOR, VERSION_PATCH, VERSION_BUILD);
}

void print_version_pcanbasic(void) {
	void* handle = dlopen(PCANBASIC_LIB_FILE, RTLD_LAZY);

	if (handle) {
		typedef __u32 (*fCAN_GetValue)(__u16 channel, __u8 param, void* buf, __u32 bufsize);
		const char *dlsym_error;

		// reset errors
		dlerror();
		fCAN_GetValue pCAN_GetValue = (fCAN_GetValue) dlsym(handle, PCANBASIC_GETVALUE);
		dlsym_error = dlerror();
		if (dlsym_error == NULL) {
			char buf[255];
			__u32 sts;
			buf[0] = 0;
			sts = pCAN_GetValue(0, PCAN_API_VERSION, buf, sizeof(buf));
			if (sts == 0) {
				fprintf(stdout, "PCAN-Basic version: %s\n", buf);
			}
			else {
				fprintf(stderr, "Failed to read PCAN-Basic version (0x%04X).\n", sts);
			}
		}
		else {
			fprintf(stderr, "Failed to load PCAN-Basic symbol => %s\n", dlsym_error);
		}
		dlclose(handle);
	} else {
		fprintf(stderr, "Failed to open PCAN-Basic library => %s\n", dlerror());
		return;
	}
}

int main(int argc, char * argv[]) {
	PCANLOG_LEVEL log_lvl;
	char * device;
	int c;
	struct pcaninfo_list *pcilist;
	int i, j, ires, doprint;
	char buf[BUF_SIZE];
	TPCANHandle hdl;

	/* get exec name */
	exec_name = strrchr(argv[0], '/');
	if (!exec_name)
		exec_name = argv[0];
	else
		++exec_name;
	/* initialization */
	device = NULL;
	log_lvl = LVL_NORMAL;
	resume_flag = 1;

	/* parse command arguments */
	while (1) {
		int option_index = 0;
		c = getopt_long(argc, argv, "d:hgval", long_options, &option_index);
		/* Detect the end of the options. */
		if (c == -1)
			break;
		switch (c) {
		case 0:
			/* If this option set a flag, do nothing else now. */
			if (long_options[option_index].flag != 0)
				break;
			break;
		case 'a':
			resume_flag = 0;
			break;
		case 'l':
			resume_flag = 1;
			break;
		case 'g':
			debug_flag = 1;
			break;
		case 'v':
			verbose_flag = 1;
			break;
		case 'h':
			print_info();
			print_version();
			print_usage(0);
			print_help();
			exit(0);
		default:
			print_usage(0);
			print_help();
			exit(0);
		}
	}

	if (verbose_flag)
		log_lvl = LVL_VERBOSE;
	if (debug_flag)
		log_lvl = LVL_DEBUG;
	pcanlog_set(log_lvl, 0, log_lvl == LVL_DEBUG);

	/* assume remaining command line arguments are devices
	 * to look for and output */
	ires = pcaninfo_get(&pcilist, 1);
	if (ires != 0) {
		return -1;
	}
	if (pcilist->version[0] != 0)
		fprintf(stdout, "PCAN driver version: %s\n", pcilist->version);
	else
		fprintf(stdout, "PCAN driver not found\n");
	print_version_pcanbasic();
	fprintf(stdout, "\n");

	/* try to match arg and existing device name */
	int print_count = 0;
	for (i = 0; i < pcilist->length; i++) {
		if (optind >= argc)
			doprint = 1;
		else {
			/* search if device is requested in args */
			doprint = 0;
			for (j = optind; j < argc; j++) {
				device = argv[j];
				/* filter on device name? */
				if (strstr(pcilist->infos[i].name, device) != 0) {
					doprint = 1;
				}
				/* filter on device path? */
				else if (strstr(pcilist->infos[i].path, device) != 0) {
					doprint = 1;
				}
				/* filter on PCAN-Basic handle? */
				else {
					hdl = pcanbasic_get_handle(pcilist->infos[i].path, pcilist);
					if (strstr(pretty_tpcanhandle(hdl, buf, BUF_SIZE), device) != 0) {
						doprint = 1;
					}
				}
			}
		}
		if (doprint) {
			print_count++;
			hdl = pcanbasic_get_handle(pcilist->infos[i].path, pcilist);
			if (resume_flag) {
				char str_hdl[sizeof(buf) + 20];
				snprintf(str_hdl, sizeof(str_hdl), "\"%s\" (0x%03x)", pretty_tpcanhandle(hdl, buf, BUF_SIZE), hdl);
				pcaninfo_output_summary(&pcilist->infos[i], str_hdl);
			}
			else {
				pcaninfo_output(&pcilist->infos[i]);
				fprintf(stdout, "  \t- TPCANHandle: \"%s\" (0x%03x)\n", pretty_tpcanhandle(hdl, buf, BUF_SIZE), hdl);
				fprintf(stdout, "  \t-----------------\n\n");
			}
			ires = 1;
		}
	}
	if (print_count == 0) {
		if (pcilist->version[0] == 0) {
			fprintf(stdout, "Warning: check PCAN driver is correctly installed and loaded.\n");
		} else {
			if (pcilist->length > 0) {
				fprintf(stdout, "Warning: requested devices not found!\n");
				print_usage(0);
			}
			else {			
				fprintf(stdout, "No device found!\n");
			}
		}
	}

	free(pcilist);

	return 0;
}
