/* SPDX-License-Identifier: LGPL-2.1-only */
/*
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
#include "peak_can_basic_PCANBasic.h"

#include "pcanbasic_jni.h"
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <stdint.h>
#include <sys/eventfd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h> 
#include "VersionNo.h"

#define true 1
#define false 0

#define LIB_PCANBASIC_NAME	"PCANBasic"

#define JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION	"java/lang/NullPointerException"
#define JAVA_CLASS_JRE_STRINGBUFFER				"java/lang/StringBuffer"
#define JAVA_CLASS_JRE_UNSATISFIEDLINKERROR		"java/lang/UnsatisfiedLinkError"
#define JAVA_CLASS_JRE_EXCEPTION				"java/lang/Exception"
#define JAVA_CLASS_JRE_IOEXCEPTION				"java/lang/IOException"

#define JAVA_CLASS_PEAK_MUTABLE_INTEGER			"peak/can/MutableInteger"
#define JAVA_CLASS_PEAK_MUTABLE_LONG			"peak/can/MutableLong"
#define JAVA_CLASS_PEAK_MUTABLE_TPCANHANDLE		"peak/can/basic/MutableTPCANHandle"
#define JAVA_CLASS_PEAK_RCVEVENTDISPATCHER		"peak/can/basic/RcvEventDispatcher"
#define JAVA_CLASS_PEAK_TPCANChannelInformation	"peak/can/basic/TPCANChannelInformation"
#define JAVA_CLASS_PEAK_TPCANChannelInformation_ARRAY	"[L"JAVA_CLASS_PEAK_TPCANChannelInformation";"
#define JAVA_CLASS_PEAK_TPCANDEVICE				"peak/can/basic/TPCANDevice"
#define JAVA_CLASS_PEAK_TPCANHANDLE				"peak/can/basic/TPCANHandle"
#define JAVA_CLASS_PEAK_TPCANSTATUS				"peak/can/basic/TPCANStatus"

#define TYPE_SIGNATURE_ARRAY_BYTE	"()[B"
#define TYPE_SIGNATURE_BYTE			"()B"
#define TYPE_SIGNATURE_INTEGER		"()I"
#define TYPE_SIGNATURE_LONG			"()J"
#define TYPE_SIGNATURE_SHORT		"()S"
#define TYPE_SIGNATURE_STRING		"()Ljava/lang/String;"

#ifndef MAX_PATH
#define MAX_PATH 260
#endif

// Define some debugging functions
#ifdef _DEBUG
#define printf_dbg(...)	fprintf(stderr, __VA_ARGS__)
static void printf_dbg_ts() {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	printf_dbg("[%010u.%06u] ", (unsigned int)tv.tv_sec, (unsigned int)tv.tv_usec);
}
#else
#define printf_dbg(...)
#define printf_dbg_ts()
#endif

#define NB_THREAD_MAX	0x99
struct pcbjni_chanfd {
	TPCANHandle ch;
	int fd;
};
static pthread_t g_thread_event;
static pthread_mutex_t g_mutex;
static pthread_mutex_t g_mutex_thread;
static struct pcbjni_chanfd g_threaded_channels[NB_THREAD_MAX];
static int g_thread_exit;
static int g_thread_refresh;
static int g_thread_stop_fd;
static void * CANReadThreadFunc(void* lpParam);
static void CANReadThreadFunc_evt_wake(int exit, int refresh);
static void CANReadThreadFunc_evt_flush();

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *jvm, void *reserved)
{
	printf_dbg("PCAN-Basic JNI v%s loaded, ", STRFILEVER);
#if defined(WIN64) || defined(__LP64__)
	printf_dbg("(c)2020 PEAK-System Technik GmbH - 64-Bit Version JNI\n");
#else 
	printf_dbg("(c)2020 PEAK-System Technik GmbH - 32-Bit Version JNI\n");
#endif
	return JNI_VERSION_1_2;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM *jvm, void *reserved)
{
	return;
}

// Initializes the PCANBasic
JNIEXPORT jboolean JNICALL Java_peak_can_basic_PCANBasic_initializeAPI(JNIEnv *env, jobject obj)
{
	// Unloads the current DLL
	UnloadAPI();
	// Loads the API
	LoadAPI(env);

	return bWasLoaded;
}

// Get Java Enum Value By Class Path and Constant Name
int GetClassEnumValue(JNIEnv *env, jobject* target, const char *className, const char *valueName, const char *byteCodeTypeName)
{
	jclass cls;
	jfieldID fid;

	// prevent linux issue
	if (valueName == NULL)
		valueName = "";
	printf_dbg("Calling %s(className='%s', valueName='%s', byteCodeTypeName='%s')...\n", __FUNCTION__, className, valueName, byteCodeTypeName);
	// Find JClass
	cls = (*env)->FindClass(env, className);
	if (cls == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass failed.");
		return -1;
	}
	// Get JField
	fid = (*env)->GetStaticFieldID(env, cls, valueName, byteCodeTypeName);
	if (fid == NULL) {
		printf_dbg("Error GetStaticFieldID(..) is null: %s(className='%s', valueName='%s', byteCodeTypeName='%s')...\n", __FUNCTION__, className, valueName, byteCodeTypeName);
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetStaticFieldID failed.");
		// Free Resources
		(*env)->DeleteLocalRef(env, cls);
		return -1;
	}
	// Get JObject
	*target = (*env)->GetStaticObjectField(env, cls, fid);
	// Free Resources
	(*env)->DeleteLocalRef(env, cls);
	return 0;
}

// Throws a given exception in JVM
void ThrowExByName(JNIEnv *env, const char *name, const char *msg)
{
	// Find JClass
	jclass cls = (*env)->FindClass(env, name);
	if (cls != NULL) {
		(*env)->ThrowNew(env, cls, msg);
		// Free Resources
		(*env)->DeleteLocalRef(env, cls);
	}
}


// Releases a loaded API
void UnloadAPI()
{
	bWasLoaded = true;
	bIsFdCapable = false;
}

// Instanciates/loads all functions within a loaded DLL
void LoadAPI(JNIEnv *env)
{
	bWasLoaded = true;
#ifdef NO_CANFD
	bIsFdCapable = false;
#else 
	bIsFdCapable = true;
#endif
	/* initialize thread related variables */
	memset(&g_thread_event, 0, sizeof(pthread_t));
	pthread_mutex_init(&g_mutex, NULL);
	pthread_mutex_init(&g_mutex_thread, NULL);
	memset(g_threaded_channels, PCAN_NONEBUS, sizeof(g_threaded_channels));
	/* create internal thread event (fd based) */
	g_thread_stop_fd = eventfd(0, EFD_NONBLOCK);
	if (g_thread_stop_fd < 0) {
		char buff[200];
		sprintf(buff, "Failed to create thread_stop file descriptor (errno=%d)...", errno);
		printf_dbg_ts();
		printf_dbg("%s - %s\n", __FUNCTION__, buff);
	}
}

/// <summary>
/// Initializes a PCAN Channel 
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Btr0Btr1">"The speed for the communication (BTR0BTR1 code)"</param>
/// <param name="HwType">"NON PLUG&PLAY: The type of hardware and operation mode"</param>
/// <param name="IOPort">"NON PLUG&PLAY: The I/O address for the parallel port"</param>
/// <param name="Interupt">"NON PLUG&PLAY: Interrupt number of the parallel port"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_Initialize
(JNIEnv *env, jobject obj, jobject Channel, jobject Btr0Btr1, jobject HwType, jint IOPort, jshort Interupt)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	int pBtr0Btr1;
	BYTE pHwType;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Parse Btr0Btr1
	pBtr0Btr1 = ParseEnumValueFromJava(env, &Btr0Btr1, TYPE_SIGNATURE_INTEGER);
	// Parse HwType
	pHwType = (BYTE)ParseEnumValueFromJava(env, &HwType, TYPE_SIGNATURE_BYTE);
	//Call PCANBasic Function
	status = PCBasic_Initialize(pChannel, pBtr0Btr1, pHwType, IOPort, Interupt);
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}
/// <summary>
/// Initializes a FD capable PCAN Channel 
/// </summary>
/// <param name="Channel">The handle of a FD capable PCAN Channel</param>
/// <param name="BitrateFD">The speed for the communication (FD Bitrate string)</param>
/// <remarks> See PCAN_BR_* values
/// Bitrate string must follow the following construction rules:
/// * parameters and values must be separated by '='
/// * Couples of Parameter/value must be separated by ','
/// * Following Parameter must be filled out: f_clock, data_brp, data_sjw, data_tseg1, data_tseg2,
///   nom_brp, nom_sjw, nom_tseg1, nom_tseg2.
/// * Following Parameters are optional (not used yet): data_ssp_offset, nom_samp</remarks>
/// <example>f_clock_mhz=80, nom_brp=1, nom_tset1=63, nom_tseg2=16, nom_sjw=7, data_brp=4, data_tset1=12, data_tseg2=7, data_sjw=1</example>
/// <returns>A TPCANStatus error code</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_InitializeFD
(JNIEnv *env, jobject obj, jobject Channel, jobject BitrateFD)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANBitrateFD pBitrate;

	// prevent access to NULL function pointer with non FD version
	if (bIsFdCapable == false)	{
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_InitializeFD: PCANBasic version without CAN FD support.");
		ParseTPCANStatusToJava(env, _LEGACY_PCAN_ERROR_ILLOPERATION, &jStatus);
		return jStatus;
	}
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Parse BitrateFD
	pBitrate = (TPCANBitrateFD)ParseEnumStrValueFromJava(env, &BitrateFD);
	//Call PCANBasic Function
	status = PCBasic_InitializeFd(pChannel, pBitrate);
	free(pBitrate);
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}
/// <summary>
/// Returns a descriptive text of a given TPCANStatus error 
/// code, in any desired language
/// </summary>
/// <remarks>The current languages available for translation are: 
/// Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
/// Italian (0x10) and French (0x0C)</remarks>
/// <param name="Error">"A TPCANStatus error code"</param>
/// <param name="Language">"Indicates a 'Primary language ID'"</param>
/// <param name="Buffer">"Buffer for a null terminated char array"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_GetErrorText
(JNIEnv *env, jobject obj, jobject Error, jshort Language, jobject stringBuffer)
{
	TPCANStatus status;
	char strInfo[256] = "";
	jobject jStatus;
	int pError;

	// Parse Btr0Btr1
	pError = ParseEnumValueFromJava(env, &Error, TYPE_SIGNATURE_INTEGER);
	//Call PCANBasic Function
	status = PCBasic_GetErrorText(pError, Language, strInfo);
	// Append Version Info To String Buffer
	if (status == PCAN_ERROR_OK)
		appendTextToJavaStringBuffer(env, stringBuffer, strInfo);
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}
/// <summary>
/// Uninitializes one or all PCAN Channels initialized by CAN_Initialize
/// </summary>
/// <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
/// uninitialize all initialized channels</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_Uninitialize
(JNIEnv *env, jobject obj, jobject Channel)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	//Call PCANBasic Function
	status = PCBasic_Uninitialize(pChannel);
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}
/// <summary>
/// Reads a CAN message from the receive queue of a PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsg structure buffer to store the CAN message"</param>
/// <param name="TimestampBuffer">"A TPCANTimestamp structure buffer to get
/// the reception time of the message. If this value is not desired, this parameter
/// should be passed as NULL"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_Read
(JNIEnv *env, jobject obj, jobject Channel, jobject MessageBuffer, jobject TimestampBuffer)
{
	TPCANHandle pChannel;
	TPCANMsg mymsg;
	TPCANTimestamp myRcvTime;
	TPCANStatus status;
	jobject jStatus;
	jclass cls;
	jmethodID mid;
	jbyteArray byteArray;

	// Verify MessageBuffer is not NULL
	if (MessageBuffer == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Read function: MessageBuffer is null.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Call CAN-API function
	if (TimestampBuffer != NULL)
		status = PCBasic_Read(pChannel, &mymsg, &myRcvTime);
	else
		status = PCBasic_Read(pChannel, &mymsg, NULL);
	// Copy message on success
	if (status == PCAN_ERROR_OK) {
		if (TimestampBuffer != NULL) {
			// Update Java rcvTime variable
			cls = (*env)->GetObjectClass(env, TimestampBuffer);
			if (cls == NULL) {
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetObjectClass for TPCANTimestamp failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus); 
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}
			// set millis	
			mid = (*env)->GetMethodID(env, cls, "setMillis", "(J)V");
			if (mid == NULL) {
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setMillis' for TPCANTimestamp failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}
			(*env)->CallVoidMethod(env, TimestampBuffer, mid, (jlong)myRcvTime.millis);
			// set millis_overflow	
			mid = (*env)->GetMethodID(env, cls, "setMillis_overflow", "(S)V");
			if (mid == NULL)
			{
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setMillis_overflow' for TPCANTimestamp failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}
			(*env)->CallVoidMethod(env, TimestampBuffer, mid, (jshort)myRcvTime.millis_overflow);
			// set micros	
			mid = (*env)->GetMethodID(env, cls, "setMicros", "(S)V");
			if (mid == NULL)
			{
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setMicros' for TPCANTimestamp failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}
			(*env)->CallVoidMethod(env, TimestampBuffer, mid, (jshort)myRcvTime.micros);
			(*env)->DeleteLocalRef(env, cls);
		}// endif TimeSTamp

		// Update Java message variable
		// Get Java class
		cls = (*env)->GetObjectClass(env, MessageBuffer);
		if (cls == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetObjectClass for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		// Set ID	
		mid = (*env)->GetMethodID(env, cls, "setID", "(I)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setID' for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jint)(mymsg).ID);
		// Set Type
		mid = (*env)->GetMethodID(env, cls, "setType", "(B)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setType' for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jbyte)(mymsg).MSGTYPE);
		// Set Length
		mid = (*env)->GetMethodID(env, cls, "setLength", "(B)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setLength' for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jbyte)(mymsg).LEN);
		// Create java byte array
		byteArray = (*env)->NewByteArray(env, (mymsg).LEN);
		if (byteArray == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "NewbyteArray failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->SetByteArrayRegion(env, byteArray, 0, (jbyte)(mymsg).LEN, (jbyte*)(mymsg).DATA);
		// Set Data
		mid = (*env)->GetMethodID(env, cls, "setData", "([BB)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetMethodID 'setData' for TPCANMsg failed.");
			(*env)->DeleteLocalRef(env, byteArray);
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, byteArray, (jbyte)(mymsg).LEN);
		(*env)->DeleteLocalRef(env, byteArray);
		(*env)->DeleteLocalRef(env, cls);
	}
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	return jStatus;
}

/// <summary>
/// Reads a CAN message from the receive queue of a FD capable PCAN Channel 
/// </summary>
/// <param name="Channel">"The handle of a FD capable PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsgFD structure buffer to store the CAN message"</param>
/// <param name="TimestampBuffer">"A TPCANTimestampFD buffer to get 
/// the reception time of the message. If this value is not desired, this parameter
/// should be passed as NULL"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_ReadFD
(JNIEnv * env, jobject obj, jobject Channel, jobject MessageBuffer, jobject TimestampBuffer)
{
	TPCANHandle pChannel;
	TPCANMsgFD mymsg;
	TPCANTimestampFD myRcvTime;
	TPCANStatus status;
	jobject jStatus;
	jclass cls;
	jmethodID mid;
	jbyteArray byteArray;
	jbyte len;

	// prevent access to NULL function pointer with non FD version
	if (bIsFdCapable == false) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD: PCANBasic version without CAN FD support.");
		ParseTPCANStatusToJava(env, _LEGACY_PCAN_ERROR_ILLOPERATION, &jStatus);
		return jStatus;
	}
	// Verify MessageBuffer is not NULL
	if (MessageBuffer == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: MessageBuffer is null.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Call CAN-API function
	if (TimestampBuffer != NULL)
		status = PCBasic_ReadFd(pChannel, &mymsg, &myRcvTime);
	else
		status = PCBasic_ReadFd(pChannel, &mymsg, NULL);
	// Copy message on success
	if (status == PCAN_ERROR_OK) {
		if (TimestampBuffer != NULL) {
			// Update Java rcvTime variable
			cls = (*env)->GetObjectClass(env, TimestampBuffer);
			if (cls == NULL)
			{
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetObjectClass for TPCANTimestampFD failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}

			// set timestamp	
			mid = (*env)->GetMethodID(env, cls, "setValue", "(J)V");
			if (mid == NULL)
			{
				ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetMethodID 'setValue' for TPCANTimestampFD failed.");
				ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
				(*env)->DeleteLocalRef(env, cls);
				return jStatus;
			}
			(*env)->CallVoidMethod(env, TimestampBuffer, mid, (jlong)myRcvTime);
			(*env)->DeleteLocalRef(env, cls);
		}// endif TimeSTamp

		// Update Java message variable
		cls = (*env)->GetObjectClass(env, MessageBuffer);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetObjectClass for TPCANMsgFD failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		// Set ID	
		mid = (*env)->GetMethodID(env, cls, "setID", "(I)V");
		if (mid == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetMethodID 'setID' for TPCANMsgFD failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jint)(mymsg).ID);
		// Set Type
		mid = (*env)->GetMethodID(env, cls, "setType", "(B)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetMethodID 'setType' for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jbyte)(mymsg).MSGTYPE);
		// Set DLC
		mid = (*env)->GetMethodID(env, cls, "setDlc", "(B)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetMethodID 'setDlc' for TPCANMsg failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, (jbyte)(mymsg).DLC);
		// get actual length from DLC to prepare Data
		len = GetLengthFromDLC((mymsg).DLC);
		// Create java byte array to set Data
		byteArray = (*env)->NewByteArray(env, (jbyte)len);
		if (byteArray == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: NewbyteArray failed.");
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->SetByteArrayRegion(env, byteArray, 0, (jbyte)len, (jbyte*)(mymsg).DATA);
		// Set Data
		mid = (*env)->GetMethodID(env, cls, "setData", "([BB)V");
		if (mid == NULL)
		{
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_ReadFD function: GetMethodID 'setData' for TPCANMsg failed.");
			(*env)->DeleteLocalRef(env, byteArray);
			ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
			(*env)->DeleteLocalRef(env, cls);
			return jStatus;
		}
		(*env)->CallVoidMethod(env, MessageBuffer, mid, byteArray, (jbyte)(mymsg).DLC);
		(*env)->DeleteLocalRef(env, byteArray);
		(*env)->DeleteLocalRef(env, cls);
	}
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	return jStatus;
}

/// <summary>
/// Sets the hadle of the Receive-Event for the Channel.
/// static method peak.can.basic.RcvEventDispatcher.dispatchRcvEvent is used
/// to notify each Receive-Event
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_SetRcvEvent
(JNIEnv *env, jobject obj, jobject Channel)
{
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANStatus status;
	int err, i, idx;
	pthread_attr_t attr;

	status = PCAN_ERROR_OK;
	// Save pointer to the loaded JVM
	(*env)->GetJavaVM(env, &m_vm);
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	/* look for an empty 'channel/fd' slot or
	 * if the channel has already been initialized for event-reception */
	err = 0;
	idx = -1;
	pthread_mutex_lock(&g_mutex);
	for (i = 0; i < NB_THREAD_MAX; i++) {
		if (g_threaded_channels[i].ch == pChannel) {
			/* channel already added */
			err = 1;
			idx = i;
		}
		if (g_threaded_channels[i].ch == PCAN_NONEBUS && idx == -1) {
			/* store index to initialize it later */
			idx = i;
		}
	}
	/* no error initialize empty slot */
	if (err == 0) {
		printf_dbg_ts();
		printf_dbg("%s - adding channel 0x%x to event-receive thread list...\n", __FUNCTION__, pChannel);
		g_threaded_channels[idx].ch = pChannel;
		g_threaded_channels[idx].fd = 0;
	}
	pthread_mutex_unlock(&g_mutex);
	/* check if thread exists or is alive */
	if (g_thread_event == 0) {
		/* ensure thread is joinable (default state) */
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
		/* create thread */
		err = pthread_create(&g_thread_event, &attr, &CANReadThreadFunc, NULL);
		printf_dbg_ts();
		printf_dbg("%s - creating event-receive thread (err=%d)\n", __FUNCTION__, err);
		status = (err != 0) ? PCAN_ERROR_UNKNOWN : PCAN_ERROR_OK;
	}
	else {
		/* send a event to refresh fd list */
		printf_dbg_ts();
		printf_dbg("%s - sending event to update thread...\n", __FUNCTION__);
		CANReadThreadFunc_evt_wake(0, 1);
	}
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	return jStatus;
}
/// <summary>
/// Resets the hadle of the Receive-Event for the Channel.
/// static method peak.can.basic.RcvEventDispatcher.dispatchRcvEvent is used
/// to notify each Receive-Event
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_ResetRcvEvent
(JNIEnv *env, jobject obj, jobject Channel)
{
	TPCANStatus status;
	TPCANHandle pChannel;
	jobject jStatus;
	int i, count;

	count = 0;
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	pthread_mutex_lock(&g_mutex);
	/* look for our channel and check if other channels are still using the thread */
	for (i = 0; i < NB_THREAD_MAX; i++) {
		if (g_threaded_channels[i].ch == pChannel) {
			printf_dbg_ts();
			printf_dbg("%s - removing channel 0x%x from index %d\n", __FUNCTION__, pChannel, i);
			g_threaded_channels[i].ch = PCAN_NONEBUS;
			g_threaded_channels[i].fd = 0;
		}
		if (g_threaded_channels[i].ch != PCAN_NONEBUS) {
			printf_dbg_ts();
			printf_dbg("%s - found another active channel (0x%x, index=%d) in receive-event thread\n", __FUNCTION__, g_threaded_channels[i].ch, i);
			count++;
		}
	}
	pthread_mutex_unlock(&g_mutex);
	if (g_thread_event != 0) {
		if (count == 0) {
			/* exit thread */
			printf_dbg_ts();
			printf_dbg("%s - sending event to kill thread...\n", __FUNCTION__);
			CANReadThreadFunc_evt_wake(1, 0);
			pthread_join(g_thread_event, NULL);
			g_thread_event = 0;
		}
		else {
			/* ask to refresh fd list */
			printf_dbg_ts();
			printf_dbg("%s - sending event to update thread...\n", __FUNCTION__);
			CANReadThreadFunc_evt_wake(0, 1);
		}
	}
	status = PCAN_ERROR_OK;
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

/// <summary>
/// Retrieves a PCAN Channel value
/// </summary>
/// <remarks>Parameters can be present or not according with the kind 
/// of Hardware (PCAN Channel) being used. If a parameter is not available,
/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Parameter">"The TPCANParameter parameter to get"</param>
/// <param name="Buffer">"Buffer for the parameter value"</param>
/// <param name="BufferLength">"Size in bytes of the buffer"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_GetValue
(JNIEnv *env, jobject obj, jobject Channel, jobject Parameter, jobject Buffer, jint BufferLength)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANParameter pParameter;
	jclass cls, clsArray;
	jmethodID mid;
	const char* className;
	int found = 0;
	
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Parse Parameter
	pParameter = (TPCANParameter)ParseEnumValueFromJava(env, &Parameter, TYPE_SIGNATURE_INTEGER);

	// Check Buffer's type
	if (!found) {
		className = JAVA_CLASS_PEAK_MUTABLE_INTEGER;
		cls = (*env)->FindClass(env, className);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_MUTABLE_INTEGER ") failed.");
			return NULL;
		}
		if ((*env)->IsInstanceOf(env, Buffer, cls) == JNI_TRUE)
		{
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, className);

			int intBuffer = 0;
			//Call PCANBasic Function
			status = PCBasic_GetParameter(pChannel, pParameter, &intBuffer, sizeof(intBuffer));
			//Parse int Result Into Java Integer To Maintain Reference
			mid = (*env)->GetMethodID(env, cls, "setValue", "(I)V");
			(*env)->CallVoidMethod(env, Buffer, mid, intBuffer);
		}
		(*env)->DeleteLocalRef(env, cls);
	}
	// Check Buffer's type
	if (!found) {
		className = JAVA_CLASS_JRE_STRINGBUFFER;
		cls = (*env)->FindClass(env, className);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_JRE_STRINGBUFFER ") failed.");
			return NULL;
		}
		if ((*env)->IsInstanceOf(env, Buffer, cls) == JNI_TRUE)
		{
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, className);

			char charBuffer[256] = "";
			//Call PCANBasic Function
			status = PCBasic_GetParameter(pChannel, pParameter, charBuffer, (WORD)BufferLength);
			// Append Version Info To String Buffer
			if (status == PCAN_ERROR_OK)
				appendTextToJavaStringBuffer(env, Buffer, charBuffer);
		}
		(*env)->DeleteLocalRef(env, cls);
	}
	// Check Buffer's type
	if (!found) {
		className = JAVA_CLASS_PEAK_MUTABLE_LONG;
		cls = (*env)->FindClass(env, className);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_MUTABLE_LONG ") failed.");
			return NULL;
		}
		if ((*env)->IsInstanceOf(env, Buffer, cls) == JNI_TRUE)
		{
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, className);

			long long longBuffer = 0;
			//Call PCANBasic Function
			status = PCBasic_GetParameter(pChannel, pParameter, &longBuffer, sizeof(longBuffer));
			//Parse int Result Into Java Integer To Maintain Reference
			mid = (*env)->GetMethodID(env, cls, "setValue", "(J)V");
			(*env)->CallVoidMethod(env, Buffer, mid, longBuffer);
		}
		(*env)->DeleteLocalRef(env, cls);
	}
	// Check Buffer's type
	if (!found) {
		className = JAVA_CLASS_PEAK_TPCANChannelInformation_ARRAY;
		clsArray = (*env)->FindClass(env, className);
		if (clsArray == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_TPCANChannelInformation_ARRAY ") failed.");
			return NULL;
		}
		if ((*env)->IsInstanceOf(env, Buffer, clsArray) == JNI_TRUE)
		{
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, className);

			jobject chInfoObj;
			jsize nbItems = (*env)->GetArrayLength(env, Buffer);
			int size = sizeof(TPCANChannelInformation) * nbItems;
			TPCANChannelInformation* chInfoBuffer = (TPCANChannelInformation*)malloc(size);
			//Call PCANBasic Function
			status = PCBasic_GetParameter(pChannel, pParameter, chInfoBuffer, size);
			//Update Java array TPCANChannelInformation[] 
			if (status == PCAN_ERROR_OK) {
				jmethodID mid_ctor;
				jmethodID mid_setChannelHandle;
				jmethodID mid_setDeviceType;
				jmethodID mid_setControllerNumber;
				jmethodID mid_setDeviceFeatures;
				jmethodID mid_setDeviceName;
				jmethodID mid_setDeviceId;
				jmethodID mid_setChannelCondition;
				jobject jobj;
				jstring str;

				cls = (*env)->FindClass(env, JAVA_CLASS_PEAK_TPCANChannelInformation);
				if (cls == NULL) {
					ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_TPCANChannelInformation ") failed.");
					return NULL;
				}
				mid_ctor = (*env)->GetMethodID(env, cls, "<init>", "()V");
				mid_setChannelHandle = (*env)->GetMethodID(env, cls, "setChannelHandle", "(L"JAVA_CLASS_PEAK_TPCANHANDLE";)V");
				mid_setDeviceType = (*env)->GetMethodID(env, cls, "setDeviceType", "(L"JAVA_CLASS_PEAK_TPCANDEVICE";)V");
				mid_setControllerNumber = (*env)->GetMethodID(env, cls, "setControllerNumber", "(B)V");
				mid_setDeviceFeatures = (*env)->GetMethodID(env, cls, "setDeviceFeatures", "(I)V");
				mid_setDeviceName = (*env)->GetMethodID(env, cls, "setDeviceName", "(Ljava/lang/String;)V");
				mid_setDeviceId = (*env)->GetMethodID(env, cls, "setDeviceId", "(I)V");
				mid_setChannelCondition = (*env)->GetMethodID(env, cls, "setChannelCondition", "(I)V");
				for (int i = 0; i < nbItems; ++i) {
					chInfoObj = (*env)->GetObjectArrayElement(env, Buffer, i);
					if (chInfoObj == NULL) {
						chInfoObj = (*env)->NewObject(env, cls, mid_ctor);
						(*env)->SetObjectArrayElement(env, Buffer, i, chInfoObj);
					}
					printf_dbg(" - chInfoBuffer[%d].channel_handle = 0x%02X\n", i, chInfoBuffer[i].channel_handle);
					printf_dbg("   - chInfoBuffer[%d].device_type = 0x%02X\n", i, chInfoBuffer[i].device_type);
					printf_dbg("   - chInfoBuffer[%d].controller_number = 0x%02X\n", i, chInfoBuffer[i].controller_number);
					printf_dbg("   - chInfoBuffer[%d].device_features = 0x%02X\n", i, chInfoBuffer[i].device_features);
					printf_dbg("   - chInfoBuffer[%d].device_name = '%s'\n", i, chInfoBuffer[i].device_name);
					printf_dbg("   - chInfoBuffer[%d].device_id = 0x%02X\n", i, chInfoBuffer[i].device_id);
					printf_dbg("   - chInfoBuffer[%d].channel_condition = 0x%02X\n", i, chInfoBuffer[i].channel_condition);

					ParseTPCANHandleToJava(env, chInfoBuffer[i].channel_handle, &jobj);
					(*env)->CallVoidMethod(env, chInfoObj, mid_setChannelHandle, jobj);
					(*env)->DeleteLocalRef(env, jobj);

					ParseTPCANDeviceToJava(env, chInfoBuffer[i].device_type, &jobj);
					(*env)->CallVoidMethod(env, chInfoObj, mid_setDeviceType, jobj);
					(*env)->DeleteLocalRef(env, jobj);

					(*env)->CallVoidMethod(env, chInfoObj, mid_setControllerNumber, chInfoBuffer[i].controller_number);
					(*env)->CallVoidMethod(env, chInfoObj, mid_setDeviceFeatures, chInfoBuffer[i].device_features);

					str = (*env)->NewStringUTF(env, chInfoBuffer[i].device_name);
					(*env)->CallVoidMethod(env, chInfoObj, mid_setDeviceName, str);
					(*env)->DeleteLocalRef(env, str);

					(*env)->CallVoidMethod(env, chInfoObj, mid_setDeviceId, chInfoBuffer[i].device_id);
					(*env)->CallVoidMethod(env, chInfoObj, mid_setChannelCondition, chInfoBuffer[i].channel_condition);
				}
				(*env)->DeleteLocalRef(env, cls);
			}
			if (chInfoBuffer != NULL) {
				free(chInfoBuffer);
				chInfoBuffer = NULL;
			}
		}
		(*env)->DeleteLocalRef(env, clsArray);
	}
	// Buffer's type unknown
	if (!found) {
		printf_dbg_ts();
		printf_dbg("%s - parameter is %s...\n", __FUNCTION__, "Unknown");
		status = PCAN_ERROR_ILLPARAMTYPE;
	}
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

/// <summary>
/// Configures or sets a PCAN Channel value 
/// </summary>
/// <remarks>Parameters can be present or not according with the kind 
/// of Hardware (PCAN Channel) being used. If a parameter is not available,
/// a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="Parameter">"The TPCANParameter parameter to set"</param>
/// <param name="Buffer">"Buffer with the value to be set"</param>
/// <param name="BufferLength">"Size in bytes of the buffer"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_SetValue
(JNIEnv *env, jobject obj, jobject Channel, jobject Parameter, jobject Buffer, jint BufferLength)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANParameter pParameter;
	jclass cls;
	jmethodID mid;
	jstring toStringResult;
	const char* charBuffer;
	char charBufferMax[MAX_PATH];
	int intBuffer;
	const char* className;
	int found = 0;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Parse Parameter
	pParameter = (TPCANParameter)ParseEnumValueFromJava(env, &Parameter, TYPE_SIGNATURE_INTEGER);
	// Check Buffer Is Instance Of StringBuffer
	if (!found) {
		className = JAVA_CLASS_JRE_STRINGBUFFER;
		cls = (*env)->FindClass(env, className);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_JRE_STRINGBUFFER ") failed.");
			return NULL;
		}
		if ((*env)->IsInstanceOf(env, Buffer, cls) == JNI_TRUE) {
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, className);

			// Retrieves content from the Java StringBuffer object
			mid = (*env)->GetMethodID(env, cls, "toString", TYPE_SIGNATURE_STRING);
			toStringResult = (jstring)(*env)->CallObjectMethod(env, Buffer, mid);
			// Convert from jstring to const char*
			charBuffer = (*env)->GetStringUTFChars(env, toStringResult, NULL);
			strncpy(charBufferMax, charBuffer, strnlen(charBuffer, MAX_PATH));
			charBufferMax[MAX_PATH - 1] = 0;
			// old style:
			// strcpy(charBufferMax, charBuffer);
			// Free Resource to avoid memory leak ...
			(*env)->ReleaseStringUTFChars(env, toStringResult, charBuffer);
			// Call PCANBasic Function
			status = PCBasic_SetParameter(pChannel, pParameter, charBufferMax, (WORD)BufferLength);
		}
		(*env)->DeleteLocalRef(env, cls);
	}
	// Check Buffer Is Instance Of Int
	if (!found) {
		// Parse int Result Into Java Integer To Maintain Reference
		cls = (*env)->GetObjectClass(env, Buffer);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "GetObjectClass(Buffer) failed.");
			return NULL;
		}
		mid = (*env)->GetMethodID(env, cls, "getValue", TYPE_SIGNATURE_INTEGER);
		if (mid != NULL) {
			found = 1;
			printf_dbg_ts();
			printf_dbg("%s - parameter is %s...\n", __FUNCTION__, "int");

			intBuffer = (int)(*env)->CallIntMethod(env, Buffer, mid);
			//Call PCANBasic Function
			status = PCBasic_SetParameter(pChannel, pParameter, &intBuffer, sizeof(intBuffer));
		}
		(*env)->DeleteLocalRef(env, cls);
	}
	// Buffer's type unknown
	if (!found) {
		printf_dbg_ts();
		printf_dbg("%s - parameter is %s...\n", __FUNCTION__, "Unknown");
		status = PCAN_ERROR_ILLPARAMTYPE;
	}
	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	return jStatus;
}
/// <summary>
/// Configures the reception filter. 
/// </summary>
/// <remarks>The message filter will be expanded with every call to 
/// this function. If it is desired to reset the filter, please use 
/// the CAN_SetParameter function</remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="FromID">"The lowest CAN ID to be received"</param>
/// <param name="ToID">"The highest CAN ID to be received"</param>
/// <param name="Mode">"Message type, Standard (11-bit identifier) or 
/// Extended (29-bit identifier)"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_FilterMessages
(JNIEnv *env, jobject obj, jobject Channel, jint FromID, jint ToID, jobject Mode)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANParameter pMode;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Parse Mode
	pMode = (TPCANMode)ParseEnumValueFromJava(env, &Mode, TYPE_SIGNATURE_BYTE);
	// Call PCANBasic Function
	status = PCBasic_FilterMessages(pChannel, FromID, ToID, pMode);
	// Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

/// <summary>
/// Transmits a CAN message 
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsg buffer with the message to be sent"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_Write
(JNIEnv *env, jobject obj, jobject Channel, jobject MessageBuffer)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANMsg message;
	int i;
	jclass cls;
	jmethodID mid;
	jbyteArray javaByteArray;
	jbyte *nativeByteArray;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Verify MessageBuffer is not NULL
	if (MessageBuffer == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: MessageBuffer is null.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	//Gets class
	cls = (*env)->GetObjectClass(env, MessageBuffer);
	if (cls == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetObjectClass failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	//Gets Method ID
	mid = (*env)->GetMethodID(env, cls, "getID", TYPE_SIGNATURE_INTEGER);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetMethodID('getID') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.ID = (*env)->CallIntMethod(env, MessageBuffer, mid);
	//Gets Type
	mid = (*env)->GetMethodID(env, cls, "getType", TYPE_SIGNATURE_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetMethodID('getType') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.MSGTYPE = (*env)->CallByteMethod(env, MessageBuffer, mid);
	//Gets Length
	mid = (*env)->GetMethodID(env, cls, "getLength", TYPE_SIGNATURE_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetMethodID('getLength') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.LEN = (*env)->CallByteMethod(env, MessageBuffer, mid);
	//get Data
	mid = (*env)->GetMethodID(env, cls, "getData", TYPE_SIGNATURE_ARRAY_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetMethodID('getData') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	javaByteArray = (jbyteArray)(*env)->CallObjectMethod(env, MessageBuffer, mid);
	if (javaByteArray == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: CallObjectMethod failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	//convert byte array to native byte array
	nativeByteArray = (*env)->GetByteArrayElements(env, javaByteArray, NULL);
	if (nativeByteArray == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_Write function: GetByteArrayElements failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	// Copy data
	for (i = 0; i < (message.LEN); i++) {
		(message.DATA)[i] = nativeByteArray[i];
	}
	// Calls PCANBasic Function
	status = PCBasic_Write(pChannel, &message);
	// Parses Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	(*env)->ReleaseByteArrayElements(env, javaByteArray, nativeByteArray, JNI_ABORT);
	(*env)->DeleteLocalRef(env, cls);
	return jStatus;
}

/// <summary>
/// Transmits a CAN message over a FD capable PCAN Channel
/// </summary>
/// <param name="Channel">"The handle of a FD capable PCAN Channel"</param>
/// <param name="MessageBuffer">"A TPCANMsgFD buffer with the message to be sent"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_WriteFD
(JNIEnv *env, jobject obj, jobject Channel, jobject MessageBuffer)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;
	TPCANMsgFD message;
	int i;
	jclass cls;
	jmethodID mid;
	jbyteArray javaByteArray;
	jbyte *nativeByteArray;
	jbyte len;

	// prevent access to NULL function pointer with non FD version
	if (bIsFdCapable == false) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD: PCANBasic version without CAN FD support.");
		ParseTPCANStatusToJava(env, _LEGACY_PCAN_ERROR_ILLOPERATION, &jStatus);
		return jStatus;
	}
	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Verify MessageBuffer is not NULL
	if (MessageBuffer == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: MessageBuffer is null.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	//Gets class
	cls = (*env)->GetObjectClass(env, MessageBuffer);
	if (cls == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetObjectClass failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		return jStatus;
	}
	//Gets Method ID
	mid = (*env)->GetMethodID(env, cls, "getID", TYPE_SIGNATURE_INTEGER);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetMethodID('getID') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.ID = (*env)->CallIntMethod(env, MessageBuffer, mid);
	//Gets Type
	mid = (*env)->GetMethodID(env, cls, "getType", TYPE_SIGNATURE_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetMethodID('getType') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.MSGTYPE = (*env)->CallByteMethod(env, MessageBuffer, mid);
	//Gets Length
	mid = (*env)->GetMethodID(env, cls, "getDlc", TYPE_SIGNATURE_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetMethodID('getLength') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	message.DLC = (*env)->CallByteMethod(env, MessageBuffer, mid);
	//get Data
	mid = (*env)->GetMethodID(env, cls, "getData", TYPE_SIGNATURE_ARRAY_BYTE);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetMethodID('getData') failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	javaByteArray = (jbyteArray)(*env)->CallObjectMethod(env, MessageBuffer, mid);
	if (javaByteArray == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: CallObjectMethod failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	//convert byte array to native byte array
	nativeByteArray = (*env)->GetByteArrayElements(env, javaByteArray, NULL);
	if (nativeByteArray == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "Java_peak_can_basic_PCANBasic_WriteFD function: GetByteArrayElements failed.");
		ParseTPCANStatusToJava(env, PCAN_ERROR_UNKNOWN, &jStatus);
		(*env)->DeleteLocalRef(env, cls);
		return jStatus;
	}
	// get actual length from DLC
	len = GetLengthFromDLC(message.DLC);
	for (i = 0; i < (len); i++) {
		(message.DATA)[i] = nativeByteArray[i];
	}
	// Calls PCANBasic Function
	status = PCBasic_WriteFd(pChannel, &message);
	// Parses Result
	ParseTPCANStatusToJava(env, status, &jStatus);
	(*env)->ReleaseByteArrayElements(env, javaByteArray, nativeByteArray, JNI_ABORT);
	(*env)->DeleteLocalRef(env, cls);
	return jStatus;
}

/// <summary>
/// Resets the receive and transmit queues of the PCAN Channel  
/// </summary>
/// <remarks>
/// A reset of the CAN controller is not performed.
/// </remarks>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_Reset
(JNIEnv *env, jobject obj, jobject Channel)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Call PCANBasic Function
	status = PCBasic_Reset(pChannel);
	// Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

/// <summary>
/// Gets the current status of a PCAN Channel 
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_GetStatus
(JNIEnv *env, jobject obj, jobject Channel)
{
	TPCANStatus status;
	jobject jStatus;
	TPCANHandle pChannel;

	// Parse Channel
	pChannel = (TPCANHandle)ParseEnumValueFromJava(env, &Channel, TYPE_SIGNATURE_SHORT);
	// Call PCANBasic Function
	status = PCBasic_GetStatus(pChannel);
	// Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

/// <summary>
/// Gets the current status of a PCAN Channel 
/// </summary>
/// <param name="Channel">"The handle of a PCAN Channel"</param>
/// <returns>"A TPCANStatus error code"</returns>
JNIEXPORT jobject JNICALL Java_peak_can_basic_PCANBasic_LookUpChannel
(JNIEnv* env, jobject obj, jobject Parameters, jobject FoundChannel)
{
	TPCANStatus status;
	jobject jStatus;
	jclass cls, clsStrBuf;
	jmethodID mid;
	jstring toStringResult;
	const char* charBuffer;
	char charBufferMax[MAX_PATH];

	clsStrBuf = (*env)->FindClass(env, JAVA_CLASS_JRE_STRINGBUFFER);
	if (clsStrBuf == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_JRE_STRINGBUFFER ") failed.");
		return NULL;
	}
	// Check Parameters Is Instance Of StringBuffer
	if ((*env)->IsInstanceOf(env, Parameters, clsStrBuf) == JNI_TRUE) {
		TPCANHandle foundChannel = PCAN_NONEBUS;
		jobject jChannel;
		// Retrieves content from the Java StringBuffer object
		mid = (*env)->GetMethodID(env, clsStrBuf, "toString", TYPE_SIGNATURE_STRING);
		toStringResult = (jstring)(*env)->CallObjectMethod(env, Parameters, mid);
		// Convert from jstring to const char*
		charBuffer = (*env)->GetStringUTFChars(env, toStringResult, NULL);
		strncpy(charBufferMax, charBuffer, strnlen(charBuffer, MAX_PATH));
		charBufferMax[MAX_PATH - 1] = 0;
		// Free Resource to avoid memory leak ...
		(*env)->ReleaseStringUTFChars(env, toStringResult, charBuffer);
		// Call PCANBasic Function
		status = PCBasic_LookUpChannel(charBufferMax, &foundChannel);
		
		// Convert foundChannel to Java TPCANHandle enum
		ParseTPCANHandleToJava(env, foundChannel, &jChannel);
		// Update buffer (FoundChannel is a MutableTPCANHandle)
		cls = (*env)->FindClass(env, JAVA_CLASS_PEAK_MUTABLE_TPCANHANDLE);
		if (cls == NULL) {
			ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_MUTABLE_TPCANHANDLE ") failed.");
			return NULL;
		}
		mid = (*env)->GetMethodID(env, cls, "setValue", "(L"JAVA_CLASS_PEAK_TPCANHANDLE";)V");
		(*env)->CallVoidMethod(env, FoundChannel, mid, jChannel);
		(*env)->DeleteLocalRef(env, cls);
	}
	else {
		status = PCAN_ERROR_ILLPARAMTYPE;
	}
	(*env)->DeleteLocalRef(env, clsStrBuf);


	//Parse Result
	ParseTPCANStatusToJava(env, status, &jStatus);

	return jStatus;
}

// Parse C/TPCANStatus to the corresponding Java/TPCANStatus
void ParseTPCANStatusToJava(JNIEnv *env, TPCANStatus source, jobject* target)
{
	char* constantName = NULL;

	// handle old PCAN-Basic versions (prior FD support)
	if (bIsFdCapable == false)
	{
		// these values are now in conflict with current PCAN_ERRORS
		switch (source)
		{
		case _LEGACY_PCAN_ERROR_ANYBUSERR:
			constantName = "PCAN_ERROR_ANYBUSERR";
			break;
		case _LEGACY_PCAN_ERROR_ILLDATA:
			constantName = "PCAN_ERROR_ILLDATA";
			break;
		case _LEGACY_PCAN_ERROR_INITIALIZE:
			constantName = "PCAN_ERROR_INITIALIZE";
			break;
		case _LEGACY_PCAN_ERROR_ILLOPERATION:
			constantName = "PCAN_ERROR_ILLOPERATION";
			break;
		}
	}
	// check if constantName was set during 
	if (constantName == NULL)
	{
		switch (source)
		{
		case PCAN_ERROR_OK:
			constantName = "PCAN_ERROR_OK";
			break;
		case PCAN_ERROR_XMTFULL:
			constantName = "PCAN_ERROR_XMTFULL";
			break;
		case PCAN_ERROR_OVERRUN:
			constantName = "PCAN_ERROR_OVERRUN";
			break;
		case PCAN_ERROR_BUSLIGHT:
			constantName = "PCAN_ERROR_BUSLIGHT";
			break;
		// NOTE: PCAN_ERROR_BUSWARNING == PCAN_ERROR_BUSHEAVY
		//case PCAN_ERROR_BUSWARNING:
		case PCAN_ERROR_BUSHEAVY:
			constantName = "PCAN_ERROR_BUSHEAVY";
			break;
		case PCAN_ERROR_BUSPASSIVE:
			constantName = "PCAN_ERROR_BUSPASSIVE";
			break;
		case PCAN_ERROR_BUSOFF:
			constantName = "PCAN_ERROR_BUSOFF";
			break;
		case _LEGACY_PCAN_ERROR_ANYBUSERR:
		case PCAN_ERROR_ANYBUSERR:
			constantName = "PCAN_ERROR_ANYBUSERR";
			break;
		case PCAN_ERROR_QRCVEMPTY:
			constantName = "PCAN_ERROR_QRCVEMPTY";
			break;
		case PCAN_ERROR_QOVERRUN:
			constantName = "PCAN_ERROR_QOVERRUN";
			break;
		case PCAN_ERROR_QXMTFULL:
			constantName = "PCAN_ERROR_QXMTFULL";
			break;
		case PCAN_ERROR_REGTEST:
			constantName = "PCAN_ERROR_REGTEST";
			break;
		case PCAN_ERROR_NODRIVER:
			constantName = "PCAN_ERROR_NODRIVER";
			break;
		case PCAN_ERROR_HWINUSE:
			constantName = "PCAN_ERROR_HWINUSE";
			break;
		case PCAN_ERROR_NETINUSE:
			constantName = "PCAN_ERROR_NETINUSE";
			break;
		case PCAN_ERROR_ILLHW:
			constantName = "PCAN_ERROR_ILLHW";
			break;
		case PCAN_ERROR_ILLNET:
			constantName = "PCAN_ERROR_ILLNET";
			break;
		// NOTE: PCAN_ERROR_ILLHANDLE == PCAN_ERROR_ILLCLIENT
		case PCAN_ERROR_ILLCLIENT:
			constantName = "PCAN_ERROR_ILLCLIENT";
			break;
		case PCAN_ERROR_RESOURCE:
			constantName = "PCAN_ERROR_RESOURCE";
			break;
		case PCAN_ERROR_ILLPARAMTYPE:
			constantName = "PCAN_ERROR_ILLPARAMTYPE";
			break;
		case PCAN_ERROR_ILLPARAMVAL:
			constantName = "PCAN_ERROR_ILLPARAMVAL";
			break;
		case PCAN_ERROR_UNKNOWN:
			constantName = "PCAN_ERROR_UNKNOWN";
			break;
		case PCAN_ERROR_ILLDATA:
			constantName = "PCAN_ERROR_ILLDATA";
			break;
		case PCAN_ERROR_CAUTION:
			constantName = "PCAN_ERROR_CAUTION";
			break;
		case PCAN_ERROR_INITIALIZE:
			constantName = "PCAN_ERROR_INITIALIZE";
			break;
		case PCAN_ERROR_ILLOPERATION:
			constantName = "PCAN_ERROR_ILLOPERATION";
			break;
		case PCAN_ERROR_ILLMODE:
			constantName = "PCAN_ERROR_ILLMODE";
			break;
		default:
			constantName = "PCAN_ERROR_UNKNOWN";
			break;
		}
	}

	printf_dbg("Calling %s(source='0x%02X'): constantName='%s'...\n", __FUNCTION__, source, constantName); 
	GetClassEnumValue(env, target, JAVA_CLASS_PEAK_TPCANSTATUS, constantName, "Lpeak/can/basic/TPCANStatus;");
}

// Parse C/TPCANHandle to the corresponding Java/TPCANHandle
void ParseTPCANHandleToJava(JNIEnv *env, TPCANHandle source, jobject* target)
{
	char* constantName = NULL;
	switch (source)
	{
	default:
	case PCAN_NONEBUS:
		constantName = "PCAN_NONEBUS";
		break;
	case PCAN_ISABUS1:
		constantName = "PCAN_ISABUS1";
		break;
	case PCAN_ISABUS2:
		constantName = "PCAN_ISABUS2";
		break;
	case PCAN_ISABUS3:
		constantName = "PCAN_ISABUS3";
		break;
	case PCAN_ISABUS4:
		constantName = "PCAN_ISABUS4";
		break;
	case PCAN_ISABUS5:
		constantName = "PCAN_ISABUS5";
		break;
	case PCAN_ISABUS6:
		constantName = "PCAN_ISABUS6";
		break;
	case PCAN_ISABUS7:
		constantName = "PCAN_ISABUS7";
		break;
	case PCAN_ISABUS8:
		constantName = "PCAN_ISABUS8";
		break;
	case PCAN_DNGBUS1:
		constantName = "PCAN_DNGBUS1";
		break;
	case PCAN_PCIBUS1:
		constantName = "PCAN_PCIBUS1";
		break;
	case PCAN_PCIBUS2:
		constantName = "PCAN_PCIBUS2";
		break;
	case PCAN_PCIBUS3:
		constantName = "PCAN_PCIBUS3";
		break;
	case PCAN_PCIBUS4:
		constantName = "PCAN_PCIBUS4";
		break;
	case PCAN_PCIBUS5:
		constantName = "PCAN_PCIBUS5";
		break;
	case PCAN_PCIBUS6:
		constantName = "PCAN_PCIBUS6";
		break;
	case PCAN_PCIBUS7:
		constantName = "PCAN_PCIBUS7";
		break;
	case PCAN_PCIBUS8:
		constantName = "PCAN_PCIBUS8";
		break;
	case PCAN_PCIBUS9:
		constantName = "PCAN_PCIBUS9";
		break;
	case PCAN_PCIBUS10:
		constantName = "PCAN_PCIBUS10";
		break;
	case PCAN_PCIBUS11:
		constantName = "PCAN_PCIBUS11";
		break;
	case PCAN_PCIBUS12:
		constantName = "PCAN_PCIBUS12";
		break;
	case PCAN_PCIBUS13:
		constantName = "PCAN_PCIBUS13";
		break;
	case PCAN_PCIBUS14:
		constantName = "PCAN_PCIBUS14";
		break;
	case PCAN_PCIBUS15:
		constantName = "PCAN_PCIBUS15";
		break;
	case PCAN_PCIBUS16:
		constantName = "PCAN_PCIBUS16";
		break;
	case PCAN_USBBUS1:
		constantName = "PCAN_USBBUS1";
		break;
	case PCAN_USBBUS2:
		constantName = "PCAN_USBBUS2";
		break;
	case PCAN_USBBUS3:
		constantName = "PCAN_USBBUS3";
		break;
	case PCAN_USBBUS4:
		constantName = "PCAN_USBBUS4";
		break;
	case PCAN_USBBUS5:
		constantName = "PCAN_USBBUS5";
		break;
	case PCAN_USBBUS6:
		constantName = "PCAN_USBBUS6";
		break;
	case PCAN_USBBUS7:
		constantName = "PCAN_USBBUS7";
		break;
	case PCAN_USBBUS8:
		constantName = "PCAN_USBBUS8";
		break;
	case PCAN_USBBUS9:
		constantName = "PCAN_USBBUS9";
		break;
	case PCAN_USBBUS10:
		constantName = "PCAN_USBBUS10";
		break;
	case PCAN_USBBUS11:
		constantName = "PCAN_USBBUS11";
		break;
	case PCAN_USBBUS12:
		constantName = "PCAN_USBBUS12";
		break;
	case PCAN_USBBUS13:
		constantName = "PCAN_USBBUS13";
		break;
	case PCAN_USBBUS14:
		constantName = "PCAN_USBBUS14";
		break;
	case PCAN_USBBUS15:
		constantName = "PCAN_USBBUS15";
		break;
	case PCAN_USBBUS16:
		constantName = "PCAN_USBBUS16";
		break;
	case PCAN_PCCBUS1:
		constantName = "PCAN_PCCBUS1";
		break;
	case PCAN_PCCBUS2:
		constantName = "PCAN_PCCBUS2";
		break;
	case _LEGACY_PCAN_LANBUS1:
	case PCAN_LANBUS1:
		constantName = "PCAN_LANBUS1";
		break;
	case _LEGACY_PCAN_LANBUS2:
	case PCAN_LANBUS2:
		constantName = "PCAN_LANBUS2";
		break;
	case _LEGACY_PCAN_LANBUS3:
	case PCAN_LANBUS3:
		constantName = "PCAN_LANBUS3";
		break;
	case _LEGACY_PCAN_LANBUS4:
	case PCAN_LANBUS4:
		constantName = "PCAN_LANBUS4";
		break;
	case _LEGACY_PCAN_LANBUS5:
	case PCAN_LANBUS5:
		constantName = "PCAN_LANBUS5";
		break;
	case _LEGACY_PCAN_LANBUS6:
	case PCAN_LANBUS6:
		constantName = "PCAN_LANBUS6";
		break;
	case _LEGACY_PCAN_LANBUS7:
	case PCAN_LANBUS7:
		constantName = "PCAN_LANBUS7";
		break;
	case _LEGACY_PCAN_LANBUS8:
	case PCAN_LANBUS8:
		constantName = "PCAN_LANBUS8";
		break;
	case PCAN_LANBUS9:
		constantName = "PCAN_LANBUS9";
		break;
	case PCAN_LANBUS10:
		constantName = "PCAN_LANBUS10";
		break;
	case PCAN_LANBUS11:
		constantName = "PCAN_LANBUS11";
		break;
	case PCAN_LANBUS12:
		constantName = "PCAN_LANBUS12";
		break;
	case PCAN_LANBUS13:
		constantName = "PCAN_LANBUS13";
		break;
	case PCAN_LANBUS14:
		constantName = "PCAN_LANBUS14";
		break;
	case PCAN_LANBUS15:
		constantName = "PCAN_LANBUS15";
		break;
	case PCAN_LANBUS16:
		constantName = "PCAN_LANBUS16";
		break;
	}

	printf_dbg("Calling %s(source='0x%02X'): constantName='%s'...\n", __FUNCTION__, source, constantName); 
	GetClassEnumValue(env, target, JAVA_CLASS_PEAK_TPCANHANDLE, constantName, "L"JAVA_CLASS_PEAK_TPCANHANDLE";");
}

// Parse C/TPCANHandle to the corresponding Java/TPCANHandle
void ParseTPCANDeviceToJava(JNIEnv* env, TPCANDevice source, jobject* target)
{
	char* constantName = NULL;
	switch (source)
	{
	default:
	case PCAN_NONE:
		constantName = "PCAN_NONE";
		break;
	case PCAN_PEAKCAN:
		constantName = "PCAN_PEAKCAN";
		break;
	case PCAN_ISA:
		constantName = "PCAN_ISA";
		break;
	case PCAN_DNG:
		constantName = "PCAN_DNG";
		break;
	case PCAN_PCI:
		constantName = "PCAN_PCI";
		break;
	case PCAN_USB:
		constantName = "PCAN_USB";
		break;
	case PCAN_PCC:
		constantName = "PCAN_PCC";
		break;
	case PCAN_VIRTUAL:
		constantName = "PCAN_VIRTUAL";
		break;
	case PCAN_LAN:
		constantName = "PCAN_LAN";
		break;
	}

	printf_dbg("Calling %s(source='0x%02X'): constantName='%s'...\n", __FUNCTION__, source, constantName); 
	GetClassEnumValue(env, target, JAVA_CLASS_PEAK_TPCANDEVICE, constantName, "L"JAVA_CLASS_PEAK_TPCANDEVICE";");
}


// Parse C/TPCANStatus to the corresponding Java/TPCANStatus
int ParseEnumValueFromJava(JNIEnv *env, jobject* source, const char *type)
{
	jclass cls;
	jmethodID mid;
	cls = (*env)->GetObjectClass(env, *source);
	if (cls == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "ParseEnumValueFromJava: GetObjectClass failed.");
		return -1;
	}
	// get and call method
	mid = (*env)->GetMethodID(env, cls, "getValue", type);
	(*env)->DeleteLocalRef(env, cls);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "ParseEnumValueFromJava: GetMethodID failed.");
		return -1;
	}
	return (*env)->CallIntMethod(env, *source, mid);
}
// Parse C/TPCANStatus to the corresponding Java/TPCANStatus
char* ParseEnumStrValueFromJava(JNIEnv *env, jobject* source)
{
	jclass cls;
	jmethodID mid;
	jstring str;
	const char * strValue;
	char* result;
	size_t len;

	cls = (*env)->GetObjectClass(env, *source);
	if (cls == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "ParseEnumValueFromJava: GetObjectClass failed.");
		return NULL;
	}
	// get and call method
	mid = (*env)->GetMethodID(env, cls, "getValue", TYPE_SIGNATURE_STRING);
	(*env)->DeleteLocalRef(env, cls);
	if (mid == NULL) {
		ThrowExByName(env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "ParseEnumValueFromJava: GetMethodID failed.");
		return NULL;
	}
	str = (jstring)(*env)->CallObjectMethod(env, *source, mid);
	if (str == NULL) {
		return NULL;
	}
	// copy UTF string
	strValue = (*env)->GetStringUTFChars(env, str, NULL);
	result = NULL;
	if (strValue != NULL) {
		len = strlen(strValue);
		result = calloc(strlen(strValue) + 1, sizeof(char));
		memcpy(result, strValue, len);
	}
	(*env)->ReleaseStringUTFChars(env, str, strValue);
	return result;
}

// Appends a string at the end of a Java StringBuffer
void appendTextToJavaStringBuffer(JNIEnv *env, jobject stringBuffer, char* strValue)
{
	jclass cls;
	jmethodID mid;

	// Obtain the Java StringBuffer class handle that corresponds to the 
	// Java StringBuffer object handle.
	cls = (*env)->GetObjectClass(env, stringBuffer);
	// Obtain the method ID for the StringBuffer append method which takes 
	// a StringBuffer object reference argument and returns a String object 
	// reference.
	mid = (*env)->GetMethodID(env, cls, "append", "(Ljava/lang/String;)Ljava/lang/StringBuffer;");
	if (mid != NULL) {
		// Create a new Java String object and populate it with the environment
		// variable value.  Obtain a handle to this object.
		jstring str = (*env)->NewStringUTF(env, strValue);
		// Call the StringBuffer object's append method passing the handle to 
		// the new Java String object.
		(*env)->CallObjectMethod(env, stringBuffer, mid, str);
		(*env)->DeleteLocalRef(env, str);
	}
	(*env)->DeleteLocalRef(env, cls);
}

void* CANReadThreadFunc(void* lpParam) {
	int i, j, nbfd, fd_sel;
	int fds[NB_THREAD_MAX];
	int highfd;
	TPCANHandle ch;
	TPCANStatus sts;
	fd_set set;
	JNIEnv *m_env;
	jclass cls;
	jmethodID mid;
	jobject jHandle;
	int do_exit, do_refresh;

	// We attach the current thread to the loaded JVM
	(*m_vm)->AttachCurrentThread(m_vm, (void**)&m_env, NULL);
	// Get RcvEventMediator class
	cls = (*m_env)->FindClass(m_env, JAVA_CLASS_PEAK_RCVEVENTDISPATCHER);
	if (cls == NULL) {
		ThrowExByName(m_env, JAVA_CLASS_JRE_NULL_POINTER_EXCEPTION, "FindClass(" JAVA_CLASS_PEAK_RCVEVENTDISPATCHER ") failed.");
		return NULL;
	}
	// Get static method dispatchRcvEvent
	mid = (*m_env)->GetStaticMethodID(m_env, cls, "dispatchRcvEvent",
		"(Lpeak/can/basic/TPCANHandle;)V");

	pthread_mutex_lock(&g_mutex_thread);
	g_thread_exit = 0;
	do_exit = g_thread_exit;
	pthread_mutex_unlock(&g_mutex_thread);
	printf_dbg("%s - Entering thread...\n", __FUNCTION__);
	while (!do_exit) {
		/* update list of FDs */
		pthread_mutex_lock(&g_mutex);
		nbfd = 0;
		highfd = 0;
		fds[nbfd++] = g_thread_stop_fd; /* index=0 is internal event */
		for (i = 0; i < NB_THREAD_MAX; i++) {
			if (g_threaded_channels[i].ch != PCAN_NONEBUS) {
				sts = CAN_GetValue(g_threaded_channels[i].ch, PCAN_RECEIVE_EVENT, &g_threaded_channels[i].fd, sizeof(g_threaded_channels[i].fd));
				if (sts != PCAN_ERROR_OK) {
					char buff[200];
					sprintf(buff, "PCAN-Basic JNI ERROR, failed to retrieve file descriptor. CAN_GetValue(PCAN_RECEIVE_EVENT...)=0x%x", sts);
					printf_dbg_ts();
					printf_dbg("%s - %s\n", __FUNCTION__, buff);
					ThrowExByName(m_env, JAVA_CLASS_JRE_EXCEPTION, buff);
				}
				fds[nbfd++] = g_threaded_channels[i].fd;
			}
		}
		pthread_mutex_unlock(&g_mutex);
		/* loop for CAN message until an event to refresh fd list is set */
		pthread_mutex_lock(&g_mutex_thread);
		g_thread_refresh = (nbfd > 1) ? 0 : 1;
		do_refresh = (g_thread_refresh | g_thread_exit);
		pthread_mutex_unlock(&g_mutex_thread);
		while (!do_refresh) {
			/* prepare file descriptors to listen to */
			FD_ZERO(&set);
			printf_dbg_ts();
			printf_dbg("%s - listening to file descr.: ", __FUNCTION__);
			for (i = 0; i < nbfd; i++) {
				highfd = (fds[i] > highfd) ? fds[i] : highfd;
				FD_SET(fds[i], &set);
				printf_dbg("%d, ", fds[i]);
			}
			printf_dbg("(max=%d)\n", highfd + 1);
			fd_sel = pselect(highfd + 1, &set, NULL, NULL, NULL, NULL);
			/* check if waiting failed */
			if (fd_sel < 0) {
				printf_dbg_ts();
				printf_dbg("%s - select failed with errno=%d.\n", __FUNCTION__, errno);
				char buff[100];
				switch (errno) {
				case EINTR:
					printf_dbg("%s - thread was interrupted...\n", __FUNCTION__);
					break;
				case EBADF:
					/* file descriptor closed, refresh list */
					break;
				default:
					/* unhandled error, clear and end thread */
					pthread_mutex_lock(&g_mutex);
					g_thread_exit = 1;
					g_thread_refresh = 1;
					g_thread_event = 0;
					for (i = 0; i < NB_THREAD_MAX; i++) {
						g_threaded_channels[i].ch = PCAN_NONEBUS;
						g_threaded_channels[i].fd = 0;
					}
					pthread_mutex_unlock(&g_mutex);
					sprintf(buff, "PCAN-Basic JNI IO ERROR on select (err=%d)", errno);
					printf_dbg_ts();
					printf_dbg("%s - %s\n", __FUNCTION__, buff);
					ThrowExByName(m_env, JAVA_CLASS_JRE_IOEXCEPTION, buff);
					break;
				}
			}
			if (fd_sel > 0) {
				/* handle g_thread_stop_fd event */
				if (FD_ISSET(fds[0], &set)) {
					printf_dbg_ts();
					printf_dbg("%s - received abort event...\n", __FUNCTION__);
					fd_sel--;
					CANReadThreadFunc_evt_flush();
				}
				if (fd_sel > 0) {
					printf_dbg_ts();
					printf_dbg("%s - received %d event(s)...", __FUNCTION__, fd_sel);
					for (i = 1; i < nbfd && fd_sel > 0; i++) {
						if (FD_ISSET(fds[i], &set)) {
							fd_sel--;
							/* get the channel based on the fd */
							ch = PCAN_NONEBUS;
							pthread_mutex_lock(&g_mutex);
							for (j = 0; j < NB_THREAD_MAX; j++) {
								if (g_threaded_channels[j].fd == fds[i]) {
									ch = g_threaded_channels[j].ch;
									break;
								}
							}
							pthread_mutex_unlock(&g_mutex);
							printf_dbg(" on fd=%d/ch=0x%x\n", fds[i], ch);
							if (ch != PCAN_NONEBUS) {
								//Parse Handle
								ParseTPCANHandleToJava(m_env, ch, &jHandle);
								// Call dispatchRcvEvent to notify java application
								(*m_env)->CallStaticVoidMethod(m_env, cls, mid, jHandle);
								(*m_env)->DeleteLocalRef(m_env, jHandle);
							}
						}
					}
				}
			}
			/* update local condition variable */
			pthread_mutex_lock(&g_mutex_thread);
			do_refresh = (g_thread_refresh | g_thread_exit);
			pthread_mutex_unlock(&g_mutex_thread);
		}
		/* update local condition variable */
		pthread_mutex_lock(&g_mutex_thread);
		do_exit = g_thread_exit;
		pthread_mutex_unlock(&g_mutex_thread);
	}
	(*m_env)->DeleteLocalRef(m_env, cls);
	printf_dbg_ts();
	printf_dbg("%s - Exiting thread...\n", __FUNCTION__);
	(*m_vm)->DetachCurrentThread(m_vm);
	return 0;
}

void CANReadThreadFunc_evt_wake(int exit, int refresh) {
	if (exit || refresh) {
		pthread_mutex_lock(&g_mutex_thread);
		if (exit) 
			g_thread_exit = 1;
		if (refresh) 
			g_thread_refresh = 1;
		pthread_mutex_unlock(&g_mutex_thread);
	}
	// Notify thread to stop if waiting in pselect
	uint64_t dummy = 1;
	int ret = -1;
	do {
		ret = write(g_thread_stop_fd, &dummy, sizeof(dummy));
	} while (ret == -1 && errno == EAGAIN);
	if (ret != sizeof(dummy)) {
		printf_dbg("%s - write failed with errno=%d on fd=%d.\n", __FUNCTION__, errno, g_thread_stop_fd);
	}
}

void CANReadThreadFunc_evt_flush()
{
	uint64_t dummy = 0;
	int ret = -1;
	do {
		ret = read(g_thread_stop_fd, &dummy, sizeof(dummy));
	} while (ret > 0);
}

// Get the length of a CAN FD message based on its DLC
jbyte GetLengthFromDLC(BYTE dlc)
{
	switch (dlc)
	{
	case 9:
		return 12;
	case 10:
		return 16;
	case 11:
		return 20;
	case 12:
		return 24;
	case 13:
		return 32;
	case 14:
		return 48;
	case 15:
		return 64;
	default:
		return dlc;
	}
}
