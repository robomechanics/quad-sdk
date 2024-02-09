/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * pcanbasic_jni.h - PCAN-Basic API JNI Implementation
 * $Id: pcanbasic_jni.h 12274 2021-08-30 12:20:04Z Fabrice $
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
 * Contribution: Jonathan Urban
 *               Uwe Wilhelm <u.wilhelm@peak-system.com>
 */
#ifndef _PCANBasic_JNI_
#define _PCANBasic_JNI_

 // old style PCAN channels from previous PCAN-Basic versions (prior FD support)
#define _LEGACY_PCAN_LANBUS1             0x81U  // PCAN-LAN interface, channel 1
#define _LEGACY_PCAN_LANBUS2             0x82U  // PCAN-LAN interface, channel 2
#define _LEGACY_PCAN_LANBUS3             0x83U  // PCAN-LAN interface, channel 3
#define _LEGACY_PCAN_LANBUS4             0x84U  // PCAN-LAN interface, channel 4
#define _LEGACY_PCAN_LANBUS5             0x85U  // PCAN-LAN interface, channel 5
#define _LEGACY_PCAN_LANBUS6             0x86U  // PCAN-LAN interface, channel 6
#define _LEGACY_PCAN_LANBUS7             0x87U  // PCAN-LAN interface, channel 7
#define _LEGACY_PCAN_LANBUS8             0x88U  // PCAN-LAN interface, channel 8

// Represents deprecated PCAN errors and status codes from previous PCAN-Basic versions (prior FD support)
#define _LEGACY_PCAN_ERROR_ANYBUSERR     (PCAN_ERROR_BUSLIGHT | PCAN_ERROR_BUSHEAVY | PCAN_ERROR_BUSOFF) // Mask for all bus errors
#define _LEGACY_PCAN_ERROR_ILLDATA       0x20000U  // Invalid data, function, or action.
#define _LEGACY_PCAN_ERROR_INITIALIZE    0x40000U  // Channel is not initialized
#define _LEGACY_PCAN_ERROR_ILLOPERATION  0x80000U  // Invalid operation


#include <jni.h>
#define TPCANMsg _TPCANMsg
#include <pcan.h>
#undef TPCANMsg
#include <PCANBasic.h>

#define BOOLEAN BYTE

// PCANBasic functions
#define PCBasic_Initialize		CAN_Initialize
#define PCBasic_InitializeFd	CAN_InitializeFD
#define PCBasic_Uninitialize	CAN_Uninitialize
#define PCBasic_Reset			CAN_Reset
#define PCBasic_GetStatus		CAN_GetStatus
#define	PCBasic_Read			CAN_Read
#define PCBasic_ReadFd			CAN_ReadFD
#define PCBasic_Write			CAN_Write
#define PCBasic_WriteFd			CAN_WriteFD
#define PCBasic_FilterMessages	CAN_FilterMessages
#define PCBasic_GetParameter	CAN_GetValue
#define PCBasic_SetParameter	CAN_SetValue
#define PCBasic_GetErrorText	CAN_GetErrorText
#define PCBasic_LookUpChannel	CAN_LookUpChannel

// Indicates DLL is loaded
BOOLEAN bWasLoaded;
// Indicates DLL is FD capable
BOOLEAN bIsFdCapable;

// Throws a given exception in JVM
void ThrowExByName(JNIEnv* env, const char* name, const char* msg);
// Get Java Class Enum Value
int GetClassEnumValue(JNIEnv* env, jobject* target, const char* className, const char* valueName, const char* byteCodeTypeName);
// Instanciates/loads all functions within a loaded DLL
void LoadAPI(JNIEnv* env);
// Releases a loaded API
void UnloadAPI();
// Parse ANSI-C TPCANStatus to the corresponding Java TPCANStatus
void ParseTPCANStatusToJava(JNIEnv*, TPCANStatus status, jobject* target);
// Parse ANSI-C TPCANStatus to the corresponding Java TPCANHandle
void ParseTPCANHandleToJava(JNIEnv* env, TPCANHandle source, jobject* target);
// Parse ANSI-C TPCANDevice to the corresponding Java TPCANDevice
void ParseTPCANDeviceToJava(JNIEnv* env, TPCANDevice source, jobject* target);
// Parse a Java Enum value to int
int ParseEnumValueFromJava(JNIEnv* env, jobject* source, const char* type);
// Parse a Java Enum value to LPSTR
char* ParseEnumStrValueFromJava(JNIEnv* env, jobject* source);
// Appends a string at the end of a Java StringBuffer
void appendTextToJavaStringBuffer(JNIEnv* env, jobject stringBuffer, char* value);
// Copy TPCANMsg object to a Java CANMessage object
int ParseCanMessageToJava(JNIEnv* env, TPCANMsg* msg, jobject* jCANMessage);
// Get the length of a CAN FD message based on its DLC
jbyte GetLengthFromDLC(BYTE dlc);

// Pointer to the loaded Java Virtual Machine
JavaVM* m_vm;

#endif
