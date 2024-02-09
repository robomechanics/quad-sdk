/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * @file pcaninfo.h
 * @brief Function prototypes to get information on PCAN devices
 * through the 'sysfs' file system.
 * $Id: pcaninfo.h 14983 2022-05-12 14:15:41Z Fabrice $
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

#ifndef __PCANINFO_H__
#define __PCANINFO_H__

/*
 * INCLUDES
 */
#include <sys/types.h>
#include <stdint.h>
#include <time.h>		/* time_t */

/*
 * DEFINES
 */
#define PCANINFO_MAX_CHAR_SIZE 256	/**< Max buffer size used in PCANINFO */

/**
 * @defgroup PCANINFO_FLAGS PCANINFO flag for structure parameter 'availflag'
 * Those flags states if the corresponding PCANINFO parameter is set
 *
 * @{
 */
#define PCANINFO_FLAG_INITIALIZED	(1<<0)	/**< structure is fully initialized */
#define PCANINFO_FLAG_NOM_BITRATE	(1<<1)	/**< 'nom_bitrate' parameter is defined */
#define PCANINFO_FLAG_BTR0BTR1		(1<<2)	/**< 'btr0btr1' parameter is defined */
#define PCANINFO_FLAG_CLOCK			(1<<3)	/**< 'clock' parameter is defined */
#define PCANINFO_FLAG_DATA_BITRATE	(1<<4)	/**< 'data_bitrate' parameter is defined */
#define PCANINFO_FLAG_DEV			(1<<5)	/**< 'dev' parameter is defined */
#define PCANINFO_FLAG_DEVID			(1<<6)	/**< 'devid' parameter is defined */
#define PCANINFO_FLAG_ERRORS		(1<<7)	/**< 'errors' parameter is defined */
#define PCANINFO_FLAG_HWTYPE		(1<<8)	/**< 'hwtype' parameter is defined */
#define PCANINFO_FLAG_IRQS			(1<<9)	/**< 'irqs' parameter is defined */
#define PCANINFO_FLAG_MINOR			(1<<10)	/**< 'minor' parameter is defined */
#define PCANINFO_FLAG_READ			(1<<11)	/**< 'read' parameter is defined */
#define PCANINFO_FLAG_SN			(1<<12)	/**< 'sn' parameter is defined */
#define PCANINFO_FLAG_STATUS		(1<<13)	/**< 'status' parameter is defined */
#define PCANINFO_FLAG_TYPE			(1<<14)	/**< 'type' parameter is defined */
#define PCANINFO_FLAG_WRITE			(1<<15)	/**< 'write' parameter is defined */
#define PCANINFO_FLAG_BASE			(1<<16)	/**< 'base' parameter is defined */
#define PCANINFO_FLAG_IRQ			(1<<17)	/**< 'irq' parameter is defined */
#define PCANINFO_FLAG_BUSLOAD		(1<<18)	/**< 'bus_load' parameter is defined */
#define PCANINFO_FLAG_BUSSTATE		(1<<19)	/**< 'bus_state' parameter is defined */
#define PCANINFO_FLAG_RXERR			(1<<20)	/**< 'rx_error_counter' parameter is defined */
#define PCANINFO_FLAG_TXERR			(1<<21)	/**< 'tx_error_counter' parameter is defined */
#define PCANINFO_FLAG_CTRLNB		(1<<22)	/**< 'ctrl_number' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_NB	(1<<23)	/**< 'adapter_number' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_NAME	(1<<24)	/**< 'adapter_name' parameter is defined */
#define PCANINFO_FLAG_ADAPTER_VERSION	(1<<25)	/**< 'adapter_version' parameter is defined */
#define PCANINFO_FLAG_RX_FIFO_RATIO	(1<<26)	/**< 'rx_fifo_ration' parameter is defined */
#define PCANINFO_FLAG_TX_FIFO_RATIO	(1<<27)	/**< 'tx_fifo_ration' parameter is defined */

#define PCANINFO_FLAG_EX_NOM_BRP	(1<<0)	/**< 'nom_brp' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_SJW	(1<<1)	/**< 'nom_sjw' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_TSEG1	(1<<2)	/**< 'nom_tseg1' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_TSEG2	(1<<3)	/**< 'nom_tseg2' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_BRP	(1<<4)	/**< 'data_brp' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_SJW	(1<<5)	/**< 'data_sjw' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_TSEG1	(1<<6)	/**< 'data_tseg1' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_TSEG2	(1<<7)	/**< 'data_tseg2' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_SAMPLE_POINT	(1<<8)	/**< 'data_sample_point' parameter is defined */
#define PCANINFO_FLAG_EX_DATA_TQ	(1<<9)			/**< 'data_tq' parameter is defined */
#define PCANINFO_FLAG_EX_DEV_NAME	(1<<10)			/**< 'dev_name' parameter is defined */
#define PCANINFO_FLAG_EX_INIT_FLAGS	(1<<11)			/**< 'init_flags' parameter is defined */
#define PCANINFO_FLAG_EX_MASS_STORAGE_MODE	(1<<12)	/**< 'mass_storage_mode' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_SAMPLE_POINT	(1<<13)	/**< 'nom_sample_point' parameter is defined */
#define PCANINFO_FLAG_EX_NOM_TQ		(1<<14)			/**< 'nom_tq' parameter is defined */
#define PCANINFO_FLAG_EX_TS_FIXED	(1<<15)			/**< 'ts_fixed' parameter is defined */
#define PCANINFO_FLAG_EX_CLK_DRIFT	(1<<16)			/**< 'clk_drift' parameter is defined */
#define PCANINFO_FLAG_EX_ADAPTER_PARTNUM (1<<17)	/**< 'adapter_partnum' parameter is defined */

/** @} */

/**
 * Defines the number of available categories in PCANINFO Hardware
 */
#define PCANINFO_HW_COUNT	9
/**
 * PCANINFO_HW PCANINFO Hardware category defines general
 * hardware categories for PCAN hardware in order to simplify mappings
 * with PCANBasic hardware definitions (TPCANDevice).
 */
enum pcaninfo_hw {
	PCANINFO_HW_NONE 	= 0x00U,	/**< Undefined, unknown or not selected PCAN device value */
	PCANINFO_HW_PEAKCAN	= 0x01U,	/**< PCAN Non-Plug&Play devices */
	PCANINFO_HW_ISA		= 0x02U,	/**< PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus */
	PCANINFO_HW_DNG		= 0x03U,	/**< PCAN-Dongle */
	PCANINFO_HW_PCI		= 0x04U,	/**< PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express and similar FD products */
	PCANINFO_HW_USB		= 0x05U,	/**< PCAN-USB and PCAN-USB Pro and similar FD products */
	PCANINFO_HW_PCC		= 0x06U,	/**< PCAN-PC Card */
	PCANINFO_HW_VIRTUAL	= 0x07U,	/**< PCAN Virtual hardware */
	PCANINFO_HW_LAN		= 0x08U		/**< PCAN Gateway devices */
};

/**
 * Stores information on a PCAN Device
 */
struct pcaninfo {
	char * classpath;						/**< Device's class path (const char no need to be freed) */
	char name[PCANINFO_MAX_CHAR_SIZE];		/**< Device name */
	char path[PCANINFO_MAX_CHAR_SIZE + 5];	/**< Device path */
	uint32_t availflag; 					/**< Each bit defines if a parameter is set (see PCANINFO_FLAG_xx) */
	uint32_t availflag_ex; 					/**< Each bit defines if a parameter is set (see PCANINFO_FLAG_EX_xx) */
	uint32_t base;							/**< I/O port for non plug'n play harware */
	uint32_t nom_bitrate;					/**< Nominal bitrate */
	uint32_t nom_brp;						/**< Nominal Bit rate point */
	uint32_t nom_sjw;						/**< Nominal SyncWidthJump */
	uint32_t nom_tseg1;						/**< Nominal Tseg1 */
	uint32_t nom_tseg2;						/**< Nominal Tseg2 */
	uint32_t nom_sample_point;				/**< Nominal Sample Point */
	uint32_t nom_tq;						/**< Nominal Time Quanta */
	uint32_t btr0btr1;						/**< Nominal bitrate as BTR0BTR1 value*/
	uint32_t clock;							/**< Device's clock frequency */
	uint32_t clk_drift;						/**< Device's clock drift */
	uint32_t clk_ref;						/**< Device's clock reference */
	uint32_t data_bitrate;					/**< Data bitrate (FD only) */
	uint32_t data_brp;						/**< Data Bit rate point (FD only) */
	uint32_t data_sjw;						/**< Data SyncWidthJump (FD only) */
	uint32_t data_tseg1;					/**< Data Tseg1 (FD only) */
	uint32_t data_tseg2;					/**< Data Tseg2 (FD only) */
	uint32_t data_sample_point;				/**< Data Sample Point */
	uint32_t data_tq;						/**< Data Time Quanta */
	char dev[PCANINFO_MAX_CHAR_SIZE];		/**< Unix device ID */
	char dev_name[PCANINFO_MAX_CHAR_SIZE+5];/**< Full device path */
	uint32_t devid;							/**< PCAN channel device ID */
	uint32_t errors;						/**< Number of CAN errors */
	uint32_t hwtype;						/**< Hardware type code */
	uint32_t init_flags;					/**< Flags used to initialize connection */
	uint32_t irq;							/**< Interrupt for non plug'n play harware */
	uint32_t irqs;							/**< Number of interrupts */
	uint32_t mass_storage_mode;				/**< Status of the Mass Storage mode */
	uint32_t minor;							/**< Unix device minor */

	uint32_t read;							/**< Number of CAN frames read */
	uint32_t sn;							/**< Serial Number */
	uint32_t status;						/**< CAN bus status */
	char type[PCANINFO_MAX_CHAR_SIZE];		/**< Hardware type as a string*/
	uint32_t write;							/**< Number of CAN frames written */
	uint32_t bus_load;						/**< Bus load */
	uint32_t bus_state;						/**< Bus state */
	uint32_t rxerr;							/**< Rx error counter */
	uint32_t txerr;							/**< Tx error counter */
	uint32_t ctrlnb;						/**< Controller number */
	uint32_t adapter_nb;					/**< Adapter number */
	char adapter_name[PCANINFO_MAX_CHAR_SIZE];	/**< Adapter name */
	char adapter_partnum[PCANINFO_MAX_CHAR_SIZE];	/**< Adapter part number */
	char adapter_version[PCANINFO_MAX_CHAR_SIZE];	/**< Adapter version */
	uint32_t rx_fifo_ratio;					/**< Filling ratio of the rx queue */
	uint32_t tx_fifo_ratio;					/**< Filling ratio of the tx queue */
	uint32_t ts_fixed;						/**< Number of timestamp fixed */

	time_t time_update;
	enum pcaninfo_hw hwcategory;			/**< Hardware category code */
	uint32_t hwindex;						/**< Index of the device for its hardware category */
};

/**
 * A list of PCANINFO structures along with its actual size
 */
struct pcaninfo_list {
	int length;			/**< Length of the array infos */
	char version[24];	/**< String version of the PCAN driver */
	struct pcaninfo infos[0];	/**< Array of PCANINFO structure */
};


/**
 * Flag definitions for struct pcaninfo_version
 */
enum pcaninfo_version_flag {
	PCB_VERSION_BUILD_SET 	= 0x01,	/**< Build's version number parsed */
	PCB_VERSION_PATCH_SET 	= 0x02,	/**< Patch's version number parsed */
	PCB_VERSION_MINOR_SET 	= 0x04,	/**< Minor's version number parsed */
	PCB_VERSION_MAJOR_SET 	= 0x08,	/**< Major's version number parsed */

	PCB_VERSION_DEFINED_MIN	= PCB_VERSION_MAJOR_SET | PCB_VERSION_MINOR_SET,
	PCB_VERSION_DEFINED_FULL = PCB_VERSION_DEFINED_MIN | PCB_VERSION_PATCH_SET
};
/**
 * Version information
 */
struct pcaninfo_version {
	int major;
	int minor;
	int patch;
	int build;
	enum pcaninfo_version_flag status;
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @fn int pcaninfo_update(PCANINFO * pci)
 * @brief Updates data fields in a PCANINFO structure
 * (classpath and name must be initialized).
 *
 * @param[in, out] pci Pointer to the PCANINFO structure to update
 * @return Status error code (0 if no error)
 */
int pcaninfo_update(struct pcaninfo * pci);

/**
 * @fn int pcaninfo_get(struct pcaninfo_list ** pcilist)
 * @brief Retrieves available PCAN devices' information.
 *
 * @param[out] pcilist buffer to store PCAN devices' information
 * @param[in] do_init state if the PCANINFO should be fully initialized (value=1)
 * 		otherwise (value=0) only the following members are valid:
 * 		'classpath', 'name'.
 * @return Status error code (0 if no error)
 */
int pcaninfo_get(struct pcaninfo_list ** pcilist, int do_init);

/**
 * @fn void pcaninfo_output(PCANINFO * pci)
 * @brief Prints a PCANINFO structure to std output
 *
 * @param[in] pci Pointer to the PCANINFO structure to output
 */
void pcaninfo_output(struct pcaninfo * pci);

/**
 * @fn void pcaninfo_output_summary(PCANINFO * pci, char* handle_info)
 * @brief Prints a summarized version of a PCANINFO structure to std output
 *
 * @param[in] pci Pointer to the PCANINFO structure to output
 * @param[in] handle_info A string describing TPCANHandle information
 */
void pcaninfo_output_summary(struct pcaninfo * pci, char* handle_info);

/**
 * @fn int pcaninfo_print(void)
 * @brief Discovers PCAN devices and prints to std output their information
 *
 * @return Status error code (0 if no error)
 */
int pcaninfo_print(void);

/**
 * @fn pcaninfo_driver_version(char *buffer, unsigned int size)
 * @brief Gets the version of the PCAN driver
 *
 * @param[out] buffer a buffer to store the version as a string
 * @param[in] size size of the buffer (15*char is enough)
 * @return Status error code (0 if no error)
 */
int pcaninfo_driver_version(char *buffer, unsigned int size);

/**
* @fn pcaninfo_bitrate_to_string(struct pcaninfo * pci, char *buffer, uint size)
* @brief Returns the full bitrate information as a formatted string
*
* @param[in] pci pointer to the PCANINFO structure to output
* @param[out] buffer a buffer to store the bitrate information as a string
* @param[in] size size of the buffer
* @return the formatted buffer
*/
char* pcaninfo_bitrate_to_string(struct pcaninfo * pci, char *buffer, unsigned int size);

/**
* @fn pcaninfo_bitrate_to_init_string(struct pcaninfo * pci, char *buffer, uint size)
* @brief Returns the full bitrate initialization string
*
* @param[in] pci pointer to the PCANINFO structure to output
* @param[out] buffer a buffer to store the bitrate information as a string
* @param[in] size size of the buffer
* @return the formatted buffer
*/
char* pcaninfo_bitrate_to_init_string(struct pcaninfo * pci, char *buffer, unsigned int size);

/**
 * @fn pcaninfo_hw_to_string(enum pcaninfo_hw hw)
 * @brief Returns the hardware type as a string
 * 
 * @param[in] hw hardware's type
 * @return the hardware's type as a formatted string
 */
const char* pcaninfo_hw_to_string(enum pcaninfo_hw hw, int no_bus_trailing);

/**
 * @fn pcaninfo_parse_version(char* version, struct pcaninfo_version* buffer)
 * @brief Returns a standard errno
 * 
 * @param[in] version version as a string 
 * @param[out] buffer a buffer to store the information parsed from the version string
 * @return 0 on success, otherwise a standard errno.
 */
int pcaninfo_parse_version(const char* version, struct pcaninfo_version* buffer);


#ifdef __cplusplus
};
#endif

#endif /* __PCANINFO_H__ */
