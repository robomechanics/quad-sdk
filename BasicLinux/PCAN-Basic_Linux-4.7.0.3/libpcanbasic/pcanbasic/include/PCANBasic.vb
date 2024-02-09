'  PCANBasic.cs
'
'  ~~~~~~~~~~~~
'
'  PCAN-Basic API
'
'  ~~~~~~~~~~~~
'
'  ------------------------------------------------------------------
'  Author : Keneth Wagner
'  Last change: 2022-11-17
'
'  Language: VB .NET
'  ------------------------------------------------------------------
'
'  Copyright (C) 1999-2022  PEAK-System Technik GmbH, Darmstadt
'  more Info at http://www.peak-system.com 
'
Imports System
Imports System.Text
Imports System.Runtime.InteropServices

Imports TPCANHandle = System.UInt16
Imports TPCANBitrateFD = System.String
Imports TPCANTimestampFD = System.UInt64

Namespace Peak.Can.Basic
#Region "Enumerations"
    ''' <summary>
    ''' Represents a PCAN status/error code
    ''' </summary>
    <Flags()> _
    Public Enum TPCANStatus As UInt32
        ''' <summary>
        ''' No error
        ''' </summary>
        PCAN_ERROR_OK = &H0
        ''' <summary>
        ''' Transmit buffer in CAN controller is full
        ''' </summary>
        PCAN_ERROR_XMTFULL = &H1
        ''' <summary>
        ''' CAN controller was read too late
        ''' </summary>
        PCAN_ERROR_OVERRUN = &H2
        ''' <summary>
        ''' Bus error: an error counter reached the 'light' limit
        ''' </summary>
        PCAN_ERROR_BUSLIGHT = &H4
        ''' <summary>
        ''' Bus error: an error counter reached the 'heavy' limit
        ''' </summary>
        PCAN_ERROR_BUSHEAVY = &H8
        ''' <summary>
        ''' Bus error: an error counter reached the 'warning' limit
        ''' </summary>
        PCAN_ERROR_BUSWARNING = PCAN_ERROR_BUSHEAVY
        ''' <summary>
        ''' Bus error: the CAN controller is error passive
        ''' </summary>
        PCAN_ERROR_BUSPASSIVE = &H40000
        ''' <summary>
        ''' Bus error: the CAN controller is in bus-off state
        ''' </summary>
        PCAN_ERROR_BUSOFF = &H10
        ''' <summary>
        ''' Mask for all bus errors
        ''' </summary>
        PCAN_ERROR_ANYBUSERR = (PCAN_ERROR_BUSWARNING Or PCAN_ERROR_BUSLIGHT Or PCAN_ERROR_BUSHEAVY Or PCAN_ERROR_BUSOFF Or PCAN_ERROR_BUSPASSIVE)
        ''' <summary>
        ''' Receive queue is empty
        ''' </summary>
        PCAN_ERROR_QRCVEMPTY = &H20
        ''' <summary>
        ''' Receive queue was read too late
        ''' </summary>
        PCAN_ERROR_QOVERRUN = &H40
        ''' <summary>
        ''' Transmit queue is full
        ''' </summary>
        PCAN_ERROR_QXMTFULL = &H80
        ''' <summary>
        ''' Test of the CAN controller hardware registers failed (no hardware found)
        ''' </summary>
        PCAN_ERROR_REGTEST = &H100
        ''' <summary>
        ''' Driver not loaded
        ''' </summary>
        PCAN_ERROR_NODRIVER = &H200
        ''' <summary>
        ''' Hardware already in use by a Net
        ''' </summary>
        PCAN_ERROR_HWINUSE = &H400
        ''' <summary>
        ''' A Client is already connected to the Net
        ''' </summary>
        PCAN_ERROR_NETINUSE = &H800
        ''' <summary>
        ''' Hardware handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLHW = &H1400
        ''' <summary>
        ''' Net handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLNET = &H1800
        ''' <summary>
        ''' Client handle is invalid
        ''' </summary>
        PCAN_ERROR_ILLCLIENT = &H1C00
        ''' <summary>
        ''' Mask for all handle errors
        ''' </summary>
        PCAN_ERROR_ILLHANDLE = (PCAN_ERROR_ILLHW Or PCAN_ERROR_ILLNET Or PCAN_ERROR_ILLCLIENT)
        ''' <summary>
        ''' Resource (FIFO, Client, timeout) cannot be created
        ''' </summary>
        PCAN_ERROR_RESOURCE = &H2000
        ''' <summary>
        ''' Invalid parameter
        ''' </summary>
        PCAN_ERROR_ILLPARAMTYPE = &H4000
        ''' <summary>
        ''' Invalid parameter value
        ''' </summary>
        PCAN_ERROR_ILLPARAMVAL = &H8000
        ''' <summary>
        ''' Unknown error
        ''' </summary>
        PCAN_ERROR_UNKNOWN = &H10000
        ''' <summary>
        ''' Invalid data, function, or action.
        ''' </summary>
        PCAN_ERROR_ILLDATA = &H20000
        ''' <summary>
        ''' Driver object state is wrong for the attempted operation
        ''' </summary>
        PCAN_ERROR_ILLMODE = &H80000
        ''' <summary>
        ''' An operation was successfully carried out, however, irregularities were registered
        ''' </summary>
        PCAN_ERROR_CAUTION = &H2000000
        ''' <summary>
        ''' Channel is not initialized
        ''' <remarks>Value was changed from 0x40000 to 0x4000000</remarks>
        ''' </summary>
        PCAN_ERROR_INITIALIZE = &H4000000
        ''' <summary>
        ''' Invalid operation
        ''' <remarks>Value was changed from 0x80000 to 0x8000000</remarks>
        ''' </summary>
        PCAN_ERROR_ILLOPERATION = &H8000000
    End Enum

    ''' <summary>
    ''' Represents a PCAN device
    ''' </summary>
    Public Enum TPCANDevice As Byte
        ''' <summary>
        ''' Undefined, unknown or not selected PCAN device value
        ''' </summary>
        PCAN_NONE = 0
        ''' <summary>
        ''' PCAN Non-PnP devices. NOT USED WITHIN PCAN-Basic API
        ''' </summary>        
        PCAN_PEAKCAN = 1
        ''' <summary>
        ''' PCAN-ISA, PCAN-PC/104, and PCAN-PC/104-Plus
        ''' </summary>
        PCAN_ISA = 2
        ''' <summary>
        ''' PCAN-Dongle
        ''' </summary>
        PCAN_DNG = 3
        ''' <summary>
        ''' PCAN-PCI, PCAN-cPCI, PCAN-miniPCI, and PCAN-PCI Express
        ''' </summary>
        PCAN_PCI = 4
        ''' <summary>
        ''' PCAN-USB and PCAN-USB Pro
        ''' </summary>
        PCAN_USB = 5
        ''' <summary>
        ''' PCAN-PC Card
        ''' </summary>
        PCAN_PCC = 6
        ''' <summary>
        ''' PCAN Virtual hardware. NOT USED WITHIN PCAN-Basic API
        ''' </summary>
        PCAN_VIRTUAL = 7
        ''' <summary>
        ''' PCAN Gateway devices
        ''' </summary>
        PCAN_LAN = 8
    End Enum

    ''' <summary>
    ''' Represents a PCAN parameter to be read or set
    ''' </summary>
    Public Enum TPCANParameter As Byte
        ''' <summary>
        ''' Device identifier parameter
        ''' </summary>
        PCAN_DEVICE_ID = 1
        ''' <summary>
        ''' DEPRECATED. Use PCAN_DEVICE_ID instead
        ''' </summary>
        <Obsolete>
        PCAN_DEVICE_NUMBER = PCAN_DEVICE_ID
        ''' <summary>
        ''' 5-Volt power parameter
        ''' </summary>
        PCAN_5VOLTS_POWER = 2
        ''' <summary>
        ''' PCAN receive event handler parameter
        ''' </summary>
        PCAN_RECEIVE_EVENT = 3
        ''' <summary>
        ''' PCAN message filter parameter
        ''' </summary>
        PCAN_MESSAGE_FILTER = 4
        ''' <summary>
        ''' PCAN-Basic API version parameter
        ''' </summary>
        PCAN_API_VERSION = 5
        ''' <summary>
        ''' PCAN device channel version parameter
        ''' </summary>
        PCAN_CHANNEL_VERSION = 6
        ''' <summary>
        ''' PCAN Reset-On-Busoff parameter
        ''' </summary>
        PCAN_BUSOFF_AUTORESET = 7
        ''' <summary>
        ''' PCAN Listen-Only parameter
        ''' </summary>
        PCAN_LISTEN_ONLY = 8
        ''' <summary>
        ''' Directory path for log files
        ''' </summary>
        PCAN_LOG_LOCATION = 9
        ''' <summary>
        ''' Debug-Log activation status
        ''' </summary>
        PCAN_LOG_STATUS = 10
        ''' <summary>
        ''' Configuration of the debugged information (LOG_FUNCTION_***)
        ''' </summary>
        PCAN_LOG_CONFIGURE = 11
        ''' <summary>
        ''' Custom insertion of text into the log file
        ''' </summary>
        PCAN_LOG_TEXT = 12
        ''' <summary>
        ''' Availability status of a PCAN-Channel
        ''' </summary>
        PCAN_CHANNEL_CONDITION = 13
        ''' <summary>
        ''' PCAN hardware name parameter
        ''' </summary>
        PCAN_HARDWARE_NAME = 14
        ''' <summary>
        ''' Message reception status of a PCAN-Channel
        ''' </summary>
        PCAN_RECEIVE_STATUS = 15
        ''' <summary>
        ''' CAN-Controller number of a PCAN-Channel
        ''' </summary>
        PCAN_CONTROLLER_NUMBER = 16
        ''' <summary>
        ''' Directory path for PCAN trace files
        ''' </summary>
        PCAN_TRACE_LOCATION = 17
        ''' <summary>
        ''' CAN tracing activation status
        ''' </summary>
        PCAN_TRACE_STATUS = 18
        ''' <summary>
        ''' Configuration of the maximum file size of a CAN trace
        ''' </summary>
        PCAN_TRACE_SIZE = 19
        ''' <summary>
        ''' Configuration of the trace file storing mode (TRACE_FILE_***)
        ''' </summary>
        PCAN_TRACE_CONFIGURE = 20
        ''' <summary>
        ''' Physical identification of a USB based PCAN-Channel by blinking its associated LED
        ''' </summary>
        PCAN_CHANNEL_IDENTIFYING = 21
        ''' <summary>
        ''' Capabilities of a PCAN device (FEATURE_***)
        ''' </summary>
        PCAN_CHANNEL_FEATURES = 22
        ''' <summary>
        ''' Using of an existing bit rate (PCAN-View connected to a channel)
        ''' </summary>
        PCAN_BITRATE_ADAPTING = 23
        ''' <summary>
        ''' Configured bit rate as Btr0Btr1 value
        ''' </summary>
        PCAN_BITRATE_INFO = 24
        ''' <summary>
        ''' Configured bit rate as TPCANBitrateFD string
        ''' </summary>
        PCAN_BITRATE_INFO_FD = 25
        ''' <summary>
        ''' Configured nominal CAN Bus speed as Bits per seconds
        ''' </summary>
        PCAN_BUSSPEED_NOMINAL = 26
        ''' <summary>
        ''' Configured CAN data speed as Bits per seconds
        ''' </summary>
        PCAN_BUSSPEED_DATA = 27
        ''' <summary>
        ''' Remote address of a LAN channel as string in IPv4 format
        ''' </summary>
        PCAN_IP_ADDRESS = 28
        ''' <summary>
        ''' Status of the Virtual PCAN-Gateway Service 
        ''' </summary>
        PCAN_LAN_SERVICE_STATUS = 29
        ''' <summary>
        ''' Status messages reception status within a PCAN-Channel
        ''' </summary>
        PCAN_ALLOW_STATUS_FRAMES = 30
        ''' <summary>
        ''' RTR messages reception status within a PCAN-Channel
        ''' </summary>
        PCAN_ALLOW_RTR_FRAMES = 31
        ''' <summary>
        ''' Error messages reception status within a PCAN-Channel
        ''' </summary>
        PCAN_ALLOW_ERROR_FRAMES = 32
        ''' <summary>
        ''' Delay, in microseconds, between sending frames
        ''' </summary>
        PCAN_INTERFRAME_DELAY = 33
        ''' <summary>
        ''' Filter over code and mask patterns for 11-Bit messages
        ''' </summary>
        PCAN_ACCEPTANCE_FILTER_11BIT = 34
        ''' <summary>
        ''' Filter over code and mask patterns for 29-Bit messages
        ''' </summary>
        PCAN_ACCEPTANCE_FILTER_29BIT = 35
        ''' <summary>
        ''' Output mode of 32 digital I/O pin of a PCAN-USB Chip. 1: Output-Active 0 : Output Inactive
        ''' </summary>
        PCAN_IO_DIGITAL_CONFIGURATION = 36
        ''' <summary>
        ''' Value assigned to a 32 digital I/O pins of a PCAN-USB Chip
        ''' </summary>
        PCAN_IO_DIGITAL_VALUE = 37
        ''' <summary>
        ''' Value assigned to a 32 digital I/O pins of a PCAN-USB Chip - Multiple digital I/O pins to 1 = High
        ''' </summary>
        PCAN_IO_DIGITAL_SET = 38
        ''' <summary>
        ''' Clear multiple digital I/O pins to 0
        ''' </summary>
        PCAN_IO_DIGITAL_CLEAR = 39
        ''' <summary>
        ''' Get value of a single analog input pin
        ''' </summary>
        PCAN_IO_ANALOG_VALUE = 40
        ''' <summary>
        ''' Get the version of the firmware used by the device associated with a PCAN-Channel
        ''' </summary>
        PCAN_FIRMWARE_VERSION = 41
        ''' <summary>
        ''' Get the amount of PCAN channels attached to a system
        ''' </summary>
        PCAN_ATTACHED_CHANNELS_COUNT = 42
        ''' <summary>
        ''' Get information about PCAN channels attached to a system
        ''' </summary>
        PCAN_ATTACHED_CHANNELS = 43
        ''' <summary>
        ''' Echo messages reception status within a PCAN-Channel
        ''' </summary>
        PCAN_ALLOW_ECHO_FRAMES = 44
        ''' <summary>
        ''' Get the part number associated to a device
        ''' </summary>
        PCAN_DEVICE_PART_NUMBER = 45
    End Enum

    ''' <summary>
    ''' Represents the type of a PCAN message
    ''' </summary>
    <Flags()> _
    Public Enum TPCANMessageType As Byte
        ''' <summary>
        ''' The PCAN message is a CAN Standard Frame (11-bit identifier)
        ''' </summary>
        PCAN_MESSAGE_STANDARD = &H0
        ''' <summary>
        ''' The PCAN message is a CAN Remote-Transfer-Request Frame
        ''' </summary>
        PCAN_MESSAGE_RTR = &H1
        ''' <summary>
        ''' The PCAN message is a CAN Extended Frame (29-bit identifier)
        ''' </summary>
        PCAN_MESSAGE_EXTENDED = &H2
        ''' <summary>
        ''' The PCAN message represents a FD frame in terms of CiA Specs
        ''' </summary>
        PCAN_MESSAGE_FD = &H4
        ''' <summary>
        ''' The PCAN message represents a FD bit rate switch (CAN data at a higher bit rate)
        ''' </summary>
        PCAN_MESSAGE_BRS = &H8
        ''' <summary>
        ''' The PCAN message represents a FD error state indicator(CAN FD transmitter was error active)
        ''' </summary>
        PCAN_MESSAGE_ESI = &H10
        ''' <summary>
        ''' The PCAN message represents an echo CAN Frame
        ''' </summary>
        PCAN_MESSAGE_ECHO = &H20
        ''' <summary>
        ''' The PCAN message represents an error frame
        ''' </summary>
        PCAN_MESSAGE_ERRFRAME = &H40
        ''' <summary>
        ''' The PCAN message represents a PCAN status message
        ''' </summary>
        PCAN_MESSAGE_STATUS = &H80
    End Enum

    ''' <summary>
    ''' Represents a PCAN filter mode
    ''' </summary>
    Public Enum TPCANMode As Byte
        ''' <summary>
        ''' Mode is Standard (11-bit identifier)
        ''' </summary>
        PCAN_MODE_STANDARD = TPCANMessageType.PCAN_MESSAGE_STANDARD
        ''' <summary>
        ''' Mode is Extended (29-bit identifier)
        ''' </summary>
        PCAN_MODE_EXTENDED = TPCANMessageType.PCAN_MESSAGE_EXTENDED
    End Enum

    ''' <summary>
    ''' Represents a PCAN Baud rate register value
    ''' </summary>
    Public Enum TPCANBaudrate As UInt16
        ''' <summary>
        ''' 1 MBit/s
        ''' </summary>
        PCAN_BAUD_1M = &H14
        ''' <summary>
        ''' 800 kBit/s
        ''' </summary>
        PCAN_BAUD_800K = &H16
        ''' <summary>
        ''' 500 kBit/s
        ''' </summary>
        PCAN_BAUD_500K = &H1C
        ''' <summary>
        ''' 250 kBit/s
        ''' </summary>
        PCAN_BAUD_250K = &H11C
        ''' <summary>
        ''' 125 kBit/s
        ''' </summary>
        PCAN_BAUD_125K = &H31C
        ''' <summary>
        ''' 100 kBit/s
        ''' </summary>
        PCAN_BAUD_100K = &H432F
        ''' <summary>
        ''' 95,238 kBit/s
        ''' </summary>
        PCAN_BAUD_95K = &HC34E
        ''' <summary>
        ''' 83,333 kBit/s
        ''' </summary>
        PCAN_BAUD_83K = &H852B
        ''' <summary>
        ''' 50 kBit/s
        ''' </summary>
        PCAN_BAUD_50K = &H472F
        ''' <summary>
        ''' 47,619 kBit/s
        ''' </summary>
        PCAN_BAUD_47K = &H1414
        ''' <summary>
        ''' 33,333 kBit/s
        ''' </summary>
        PCAN_BAUD_33K = &H8B2F
        ''' <summary>
        ''' 20 kBit/s
        ''' </summary>
        PCAN_BAUD_20K = &H532F
        ''' <summary>
        ''' 10 kBit/s
        ''' </summary>
        PCAN_BAUD_10K = &H672F
        ''' <summary>
        ''' 5 kBit/s
        ''' </summary>
        PCAN_BAUD_5K = &H7F7F
    End Enum

    ''' <summary>
    ''' Represents the type of PCAN (Non-PnP) hardware to be initialized
    ''' </summary>
    Public Enum TPCANType As Byte
        ''' <summary>
        ''' PCAN-ISA 82C200
        ''' </summary>
        PCAN_TYPE_ISA = &H1
        ''' <summary>
        ''' PCAN-ISA SJA1000
        ''' </summary>
        PCAN_TYPE_ISA_SJA = &H9
        ''' <summary>
        ''' PHYTEC ISA 
        ''' </summary>
        PCAN_TYPE_ISA_PHYTEC = &H4
        ''' <summary>
        ''' PCAN-Dongle 82C200
        ''' </summary>
        PCAN_TYPE_DNG = &H2
        ''' <summary>
        ''' PCAN-Dongle EPP 82C200
        ''' </summary>
        PCAN_TYPE_DNG_EPP = &H3
        ''' <summary>
        ''' PCAN-Dongle SJA1000
        ''' </summary>
        PCAN_TYPE_DNG_SJA = &H5
        ''' <summary>
        ''' PCAN-Dongle EPP SJA1000
        ''' </summary>
        PCAN_TYPE_DNG_SJA_EPP = &H6
    End Enum
#End Region

#Region "Structures"
    ''' <summary>
    ''' Represents a PCAN message
    ''' </summary>
    Public Structure TPCANMsg
        ''' <summary>
        ''' 11/29-bit message identifier
        ''' </summary>
        Public ID As UInt32
        ''' <summary>
        ''' Type of the message
        ''' </summary>
        <MarshalAs(UnmanagedType.U1)> _
        Public MSGTYPE As TPCANMessageType
        ''' <summary>
        ''' Data Length Code of the message (0..8)
        ''' </summary>
        Public LEN As Byte
        ''' <summary>
        ''' Data of the message (DATA[0]..DATA[7])
        ''' </summary>
        <MarshalAs(UnmanagedType.ByValArray, SizeConst:=8)> _
        Public DATA As Byte()
    End Structure

    ''' <summary>
    ''' Represents a timestamp of a received PCAN message.
    ''' Total Microseconds = micros + 1000 * millis + 0x100000000 * 1000 * millis_overflow
    ''' </summary>
    Public Structure TPCANTimestamp
        ''' <summary>
        ''' Base-value: milliseconds: 0.. 2^32-1
        ''' </summary>
        Public millis As UInt32
        ''' <summary>
        ''' Roll-arounds of millis
        ''' </summary>
        Public millis_overflow As UInt16
        ''' <summary>
        ''' Microseconds: 0..999
        ''' </summary>
        Public micros As UInt16
    End Structure

    ''' <summary>
    ''' Represents a PCAN message from a FD capable hardware
    ''' </summary>
    Public Structure TPCANMsgFD
        ''' <summary>
        ''' 11/29-bit message identifier
        ''' </summary>
        Public ID As UInt32
        ''' <summary>
        ''' Type of the message
        ''' </summary>
        <MarshalAs(UnmanagedType.U1)>
        Public MSGTYPE As TPCANMessageType
        ''' <summary>
        ''' Data Length Code of the message (0..15)
        ''' </summary>
        Public DLC As Byte
        ''' <summary>
        ''' Data of the message (DATA[0]..DATA[63])
        ''' </summary>
        <MarshalAs(UnmanagedType.ByValArray, SizeConst:=64)>
        Public DATA As Byte()
    End Structure


    ''' <summary>
    ''' Describes an available PCAN channel
    ''' </summary>
    Public Structure TPCANChannelInformation
        ''' <summary>
        ''' PCAN channel handle
        ''' </summary>
        <MarshalAs(UnmanagedType.U2)>
        Public channel_handle As TPCANHandle
        ''' <summary>
        ''' Kind of PCAN device
        ''' </summary>
        <MarshalAs(UnmanagedType.U1)>
        Public device_type As TPCANDevice
        ''' <summary>
        ''' CAN-Controller number
        ''' </summary>
        Public controller_number As Byte
        ''' <summary>
        ''' Device capabilities flag (see FEATURE_*)
        ''' </summary>
        Public device_features As UInt32
        ''' <summary>
        ''' Device name
        ''' </summary>
        <MarshalAs(UnmanagedType.ByValTStr, SizeConst:=PCANBasic.MAX_LENGTH_HARDWARE_NAME)>
        Public device_name As String
        ''' <summary>
        ''' Device number
        ''' </summary>
        Public device_id As UInt32
        ''' <summary>
        ''' Availability status of a PCAN-Channel
        ''' </summary>
        Public channel_condition As UInt32
    End Structure
#End Region

#Region "PCANBasic class"
    ''' <summary>
    ''' PCAN-Basic API class implementation
    ''' </summary>
    Public NotInheritable Class PCANBasic
#Region "PCAN-BUS Handles Definition"
        ''' <summary>
        ''' Undefined/default value for a PCAN bus
        ''' </summary>
        Public Const PCAN_NONEBUS As TPCANHandle = &H0

        ''' <summary>
        ''' PCAN-ISA interface, channel 1
        ''' </summary>
        Public Const PCAN_ISABUS1 As TPCANHandle = &H21
        ''' <summary>
        ''' PCAN-ISA interface, channel 2
        ''' </summary>
        Public Const PCAN_ISABUS2 As TPCANHandle = &H22
        ''' <summary>
        ''' PCAN-ISA interface, channel 3
        ''' </summary>
        Public Const PCAN_ISABUS3 As TPCANHandle = &H23
        ''' <summary>
        ''' PCAN-ISA interface, channel 4
        ''' </summary>
        Public Const PCAN_ISABUS4 As TPCANHandle = &H24
        ''' <summary>
        ''' PCAN-ISA interface, channel 5
        ''' </summary>
        Public Const PCAN_ISABUS5 As TPCANHandle = &H25
        ''' <summary>
        ''' PCAN-ISA interface, channel 6
        ''' </summary>
        Public Const PCAN_ISABUS6 As TPCANHandle = &H26
        ''' <summary>
        ''' PCAN-ISA interface, channel 7
        ''' </summary>
        Public Const PCAN_ISABUS7 As TPCANHandle = &H27
        ''' <summary>
        ''' PCAN-ISA interface, channel 8
        ''' </summary>
        Public Const PCAN_ISABUS8 As TPCANHandle = &H28

        ''' <summary>
        ''' PPCAN-Dongle/LPT interface, channel 1 
        ''' </summary>
        Public Const PCAN_DNGBUS1 As TPCANHandle = &H31

        ''' <summary>
        ''' PCAN-PCI interface, channel 1
        ''' </summary>
        Public Const PCAN_PCIBUS1 As TPCANHandle = &H41
        ''' <summary>
        ''' PCAN-PCI interface, channel 2
        ''' </summary>
        Public Const PCAN_PCIBUS2 As TPCANHandle = &H42
        ''' <summary>
        ''' PCAN-PCI interface, channel 3
        ''' </summary>
        Public Const PCAN_PCIBUS3 As TPCANHandle = &H43
        ''' <summary>
        ''' PCAN-PCI interface, channel 4
        ''' </summary>
        Public Const PCAN_PCIBUS4 As TPCANHandle = &H44
        ''' <summary>
        ''' PCAN-PCI interface, channel 5
        ''' </summary>
        Public Const PCAN_PCIBUS5 As TPCANHandle = &H45
        ''' <summary>
        ''' PCAN-PCI interface, channel 6
        ''' </summary>
        Public Const PCAN_PCIBUS6 As TPCANHandle = &H46
        ''' <summary>
        ''' PCAN-PCI interface, channel 7
        ''' </summary>
        Public Const PCAN_PCIBUS7 As TPCANHandle = &H47
        ''' <summary>
        ''' PCAN-PCI interface, channel 8
        ''' </summary>
        Public Const PCAN_PCIBUS8 As TPCANHandle = &H48
        ''' <summary>
        ''' PCAN-PCI interface, channel 9
        ''' </summary>
        Public Const PCAN_PCIBUS9 As TPCANHandle = &H409
        ''' <summary>
        ''' PCAN-PCI interface, channel 10
        ''' </summary>
        Public Const PCAN_PCIBUS10 As TPCANHandle = &H40A
        ''' <summary>
        ''' PCAN-PCI interface, channel 11
        ''' </summary>
        Public Const PCAN_PCIBUS11 As TPCANHandle = &H40B
        ''' <summary>
        ''' PCAN-PCI interface, channel 12
        ''' </summary>
        Public Const PCAN_PCIBUS12 As TPCANHandle = &H40C
        ''' <summary>
        ''' PCAN-PCI interface, channel 13
        ''' </summary>
        Public Const PCAN_PCIBUS13 As TPCANHandle = &H40D
        ''' <summary>
        ''' PCAN-PCI interface, channel 14
        ''' </summary>
        Public Const PCAN_PCIBUS14 As TPCANHandle = &H40E
        ''' <summary>
        ''' PCAN-PCI interface, channel 15
        ''' </summary>
        Public Const PCAN_PCIBUS15 As TPCANHandle = &H40F
        ''' <summary>
        ''' PCAN-PCI interface, channel 16
        ''' </summary>
        Public Const PCAN_PCIBUS16 As TPCANHandle = &H410

        ''' <summary>
        ''' PCAN-USB interface, channel 1
        ''' </summary>
        Public Const PCAN_USBBUS1 As TPCANHandle = &H51
        ''' <summary>
        ''' PCAN-USB interface, channel 2
        ''' </summary>
        Public Const PCAN_USBBUS2 As TPCANHandle = &H52
        ''' <summary>
        ''' PCAN-USB interface, channel 3
        ''' </summary>
        Public Const PCAN_USBBUS3 As TPCANHandle = &H53
        ''' <summary>
        ''' PCAN-USB interface, channel 4
        ''' </summary>
        Public Const PCAN_USBBUS4 As TPCANHandle = &H54
        ''' <summary>
        ''' PCAN-USB interface, channel 5
        ''' </summary>
        Public Const PCAN_USBBUS5 As TPCANHandle = &H55
        ''' <summary>
        ''' PCAN-USB interface, channel 6
        ''' </summary>
        Public Const PCAN_USBBUS6 As TPCANHandle = &H56
        ''' <summary>
        ''' PCAN-USB interface, channel 7
        ''' </summary>
        Public Const PCAN_USBBUS7 As TPCANHandle = &H57
        ''' <summary>
        ''' PCAN-USB interface, channel 8
        ''' </summary>
        Public Const PCAN_USBBUS8 As TPCANHandle = &H58
        ''' <summary>
        ''' PCAN-USB interface, channel 9
        ''' </summary>
        Public Const PCAN_USBBUS9 As TPCANHandle = &H509
        ''' <summary>
        ''' PCAN-USB interface, channel 10
        ''' </summary>
        Public Const PCAN_USBBUS10 As TPCANHandle = &H50A
        ''' <summary>
        ''' PCAN-USB interface, channel 11
        ''' </summary>
        Public Const PCAN_USBBUS11 As TPCANHandle = &H50B
        ''' <summary>
        ''' PCAN-USB interface, channel 12
        ''' </summary>
        Public Const PCAN_USBBUS12 As TPCANHandle = &H50C
        ''' <summary>
        ''' PCAN-USB interface, channel 13
        ''' </summary>
        Public Const PCAN_USBBUS13 As TPCANHandle = &H50D
        ''' <summary>
        ''' PCAN-USB interface, channel 14
        ''' </summary>
        Public Const PCAN_USBBUS14 As TPCANHandle = &H50E
        ''' <summary>
        ''' PCAN-USB interface, channel 15
        ''' </summary>
        Public Const PCAN_USBBUS15 As TPCANHandle = &H50F
        ''' <summary>
        ''' PCAN-USB interface, channel 16
        ''' </summary>
        Public Const PCAN_USBBUS16 As TPCANHandle = &H510

        ''' <summary>
        ''' PCAN-PC Card interface, channel 1
        ''' </summary>
        Public Const PCAN_PCCBUS1 As TPCANHandle = &H61
        ''' <summary>
        ''' PCAN-PC Card interface, channel 2
        ''' </summary>
        Public Const PCAN_PCCBUS2 As TPCANHandle = &H62

        ''' <summary>
        ''' PCAN-LAN interface, channel 1
        ''' </summary>
        Public Const PCAN_LANBUS1 As TPCANHandle = &H801
        ''' <summary>
        ''' PCAN-LAN interface, channel 2
        ''' </summary>
        Public Const PCAN_LANBUS2 As TPCANHandle = &H802
        ''' <summary>
        ''' PCAN-LAN interface, channel 3
        ''' </summary>
        Public Const PCAN_LANBUS3 As TPCANHandle = &H803
        ''' <summary>
        ''' PCAN-LAN interface, channel 4
        ''' </summary>
        Public Const PCAN_LANBUS4 As TPCANHandle = &H804
        ''' <summary>
        ''' PCAN-LAN interface, channel 5
        ''' </summary>
        Public Const PCAN_LANBUS5 As TPCANHandle = &H805
        ''' <summary>
        ''' PCAN-LAN interface, channel 6
        ''' </summary>
        Public Const PCAN_LANBUS6 As TPCANHandle = &H806
        ''' <summary>
        ''' PCAN-LAN interface, channel 7
        ''' </summary>
        Public Const PCAN_LANBUS7 As TPCANHandle = &H807
        ''' <summary>
        ''' PCAN-LAN interface, channel 8
        ''' </summary>
        Public Const PCAN_LANBUS8 As TPCANHandle = &H808
        ''' <summary>
        ''' PCAN-LAN interface, channel 9
        ''' </summary>
        Public Const PCAN_LANBUS9 As TPCANHandle = &H809
        ''' <summary>
        ''' PCAN-LAN interface, channel 10
        ''' </summary>
        Public Const PCAN_LANBUS10 As TPCANHandle = &H80A
        ''' <summary>
        ''' PCAN-LAN interface, channel 11
        ''' </summary>
        Public Const PCAN_LANBUS11 As TPCANHandle = &H80B
        ''' <summary>
        ''' PCAN-LAN interface, channel 12
        ''' </summary>
        Public Const PCAN_LANBUS12 As TPCANHandle = &H80C
        ''' <summary>
        ''' PCAN-LAN interface, channel 13
        ''' </summary>
        Public Const PCAN_LANBUS13 As TPCANHandle = &H80D
        ''' <summary>
        ''' PCAN-LAN interface, channel 14
        ''' </summary>
        Public Const PCAN_LANBUS14 As TPCANHandle = &H80E
        ''' <summary>
        ''' PCAN-LAN interface, channel 15
        ''' </summary>
        Public Const PCAN_LANBUS15 As TPCANHandle = &H80F
        ''' <summary>
        ''' PCAN-LAN interface, channel 16
        ''' </summary>
        Public Const PCAN_LANBUS16 As TPCANHandle = &H810
#End Region

#Region "FD Bit rate parameters"
        ''' <summary>
        ''' Clock frequency in Herz (80000000, 60000000, 40000000, 30000000, 24000000, 20000000)
        ''' </summary>
        Public Const PCAN_BR_CLOCK As String = "f_clock"
        ''' <summary>
        ''' Clock frequency in Megaherz (80, 60, 40, 30, 24, 20)
        ''' </summary>
        Public Const PCAN_BR_CLOCK_MHZ As String = "f_clock_mhz"
        ''' <summary>
        ''' Clock prescaler for nominal time quantum
        ''' </summary>
        Public Const PCAN_BR_NOM_BRP As String = "nom_brp"
        ''' <summary>
        ''' TSEG1 segment for nominal bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_NOM_TSEG1 As String = "nom_tseg1"
        ''' <summary>
        ''' TSEG2 segment for nominal bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_NOM_TSEG2 As String = "nom_tseg2"
        ''' <summary>
        ''' Synchronization Jump Width for nominal bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_NOM_SJW As String = "nom_sjw"
        ''' <summary>
        ''' Sample point for nominal bit rate
        ''' </summary>
        Public Const PCAN_BR_NOM_SAMPLE As String = "nom_sam"
        ''' <summary>
        ''' Clock prescaler for highspeed data time quantum
        ''' </summary>
        Public Const PCAN_BR_DATA_BRP As String = "data_brp"
        ''' <summary>
        ''' TSEG1 segment for fast data bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_DATA_TSEG1 As String = "data_tseg1"
        ''' <summary>
        ''' TSEG2 segment for fast data bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_DATA_TSEG2 As String = "data_tseg2"
        ''' <summary>
        ''' Synchronization Jump Width for highspeed data bit rate in time quanta
        ''' </summary>
        Public Const PCAN_BR_DATA_SJW As String = "data_sjw"
        ''' <summary>
        ''' Secondary sample point delay for highspeed data bit rate in cyles
        ''' </summary>
        Public Const PCAN_BR_DATA_SAMPLE As String = "data_ssp_offset"
#End Region

#Region "Parameter values definition"
        ''' <summary>
        ''' The PCAN parameter is not set (inactive)
        ''' </summary>
        Public Const PCAN_PARAMETER_OFF As Integer = 0
        ''' <summary>
        ''' The PCAN parameter is set (active)
        ''' </summary>
        Public Const PCAN_PARAMETER_ON As Integer = 1
        ''' <summary>
        ''' The PCAN filter is closed. No messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_CLOSE As Integer = 0
        ''' <summary>
        ''' The PCAN filter is fully opened. All messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_OPEN As Integer = 1
        ''' <summary>
        ''' The PCAN filter is custom configured. Only registered 
        ''' messages will be received
        ''' </summary>
        Public Const PCAN_FILTER_CUSTOM As Integer = 2
        ''' <summary>
        ''' The PCAN-Channel handle is illegal, or its associated hardware is not available
        ''' </summary>
        Public Const PCAN_CHANNEL_UNAVAILABLE As Integer = 0
        ''' <summary>
        ''' The PCAN-Channel handle is available to be connected (PnP Hardware: it means furthermore that the hardware is plugged-in)
        ''' </summary>
        Public Const PCAN_CHANNEL_AVAILABLE As Integer = 1
        ''' <summary>
        ''' The PCAN-Channel handle is valid, and is already being used
        ''' </summary>
        Public Const PCAN_CHANNEL_OCCUPIED As Integer = 2
        ''' <summary>
        ''' The PCAN-Channel handle is already being used by a PCAN-View application, but is available to connect
        ''' </summary>
        Public Const PCAN_CHANNEL_PCANVIEW As Integer = PCAN_CHANNEL_AVAILABLE Or PCAN_CHANNEL_OCCUPIED

        ''' <summary>
        ''' Logs system exceptions / errors
        ''' </summary>
        Public Const LOG_FUNCTION_DEFAULT As Integer = &H0
        ''' <summary>
        ''' Logs the entries to the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_ENTRY As Integer = &H1
        ''' <summary>
        ''' Logs the parameters passed to the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_PARAMETERS As Integer = &H2
        ''' <summary>
        ''' Logs the exits from the PCAN-Basic API functions 
        ''' </summary>
        Public Const LOG_FUNCTION_LEAVE As Integer = &H4
        ''' <summary>
        ''' Logs the CAN messages passed to the CAN_Write function
        ''' </summary>
        Public Const LOG_FUNCTION_WRITE As Integer = &H8
        ''' <summary>
        ''' Logs the CAN messages received within the CAN_Read function
        ''' </summary>
        Public Const LOG_FUNCTION_READ As Integer = &H10
        ''' <summary>
        ''' Logs all possible information within the PCAN-Basic API functions
        ''' </summary>
        Public Const LOG_FUNCTION_ALL As Integer = &HFFFF

        ''' <summary>
        ''' A single file is written until it size reaches PAN_TRACE_SIZE
        ''' </summary>
        Public Const TRACE_FILE_SINGLE As Integer = &H0
        ''' <summary>
        ''' Traced data is distributed in several files with size PAN_TRACE_SIZE
        ''' </summary>
        Public Const TRACE_FILE_SEGMENTED As Integer = &H1
        ''' <summary>
        ''' Includes the date into the name of the trace file
        ''' </summary>
        Public Const TRACE_FILE_DATE As Integer = &H2
        ''' <summary>
        ''' Includes the start time into the name of the trace file
        ''' </summary>
        Public Const TRACE_FILE_TIME As Integer = &H4
        ''' <summary>
        ''' Causes the overwriting of available traces (same name)
        ''' </summary>
        Public Const TRACE_FILE_OVERWRITE As Integer = &H80
        ''' <summary>
        ''' Causes using the data length column ('l') instead of the DLC column ('L') in the trace file
        ''' </summary>
        Public Const TRACE_FILE_DATA_LENGTH As Integer = &H100

        ''' <summary>
        ''' Device supports flexible data-rate (CAN-FD)
        ''' </summary>
        Public Const FEATURE_FD_CAPABLE As Integer = &H1
        ''' <summary>
        ''' Device supports a delay between sending frames (FPGA based USB devices)
        ''' </summary>
        Public Const FEATURE_DELAY_CAPABLE As Integer = &H2
        ''' <summary>
        ''' Device supports I/O functionality for electronic circuits (USB-Chip devices)
        ''' </summary>
        Public Const FEATURE_IO_CAPABLE As Integer = &H4

        ''' <summary>
        ''' The service is not running
        ''' </summary>
        Public Const SERVICE_STATUS_STOPPED As Integer = &H1
        ''' <summary>
        ''' The service is running
        ''' </summary>
        Public Const SERVICE_STATUS_RUNNING As Integer = &H4
#End Region

#Region "Lookup Parameters"
        ''' <summary>
        ''' Lookup channel by Device type (see PCAN devices e.g. PCAN_USB)
        ''' </summary>
        Public Const LOOKUP_DEVICE_TYPE As String = "devicetype"
        ''' <summary>
        ''' Lookup channel by device id
        ''' </summary>
        Public Const LOOKUP_DEVICE_ID As String = "deviceid"
        ''' <summary>
        ''' Lookup channel by CAN controller 0-based index
        ''' </summary>
        Public Const LOOKUP_CONTROLLER_NUMBER As String = "controllernumber"
        ''' <summary>
        ''' Lookup channel by IP address (LAN channels only)
        ''' </summary>
        Public Const LOOKUP_IP_ADDRESS As String = "ipaddress"
#End Region

#Region "Other Constants"
        ''' <summary>
        ''' Maximum length of the name of a device: 32 characters + terminator
        ''' </summary>
        Public Const MAX_LENGTH_HARDWARE_NAME As Integer = 33
        ''' <summary>
        ''' ' Maximum length of a version string: 255 characters + terminator
        ''' </summary>
        Public Const MAX_LENGTH_VERSION_STRING As Integer = 256
#End Region

#Region "PCANBasic API Implementation"
        ''' <summary>
        ''' Initializes a PCAN Channel 
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        ''' <param name="HwType">Non-PnP: The type of hardware and operation mode</param>
        ''' <param name="IOPort">Non-PnP: The I/O address for the parallel port</param>
        ''' <param name="Interrupt">Non-PnP: Interrupt number of the parallel por</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Initialize")> _
        Public Shared Function Initialize( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Btr0Btr1 As TPCANBaudrate, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal HwType As TPCANType, _
            ByVal IOPort As UInt32, _
            ByVal Interrupt As UInt16) As TPCANStatus
        End Function

        ''' <summary>
        ''' Initializes a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Btr0Btr1">The speed for the communication (BTR0BTR1 code)</param>
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function Initialize( _
            ByVal Channel As TPCANHandle, _
            ByVal Btr0Btr1 As TPCANBaudrate) As TPCANStatus
            Return Initialize(Channel, Btr0Btr1, 0, 0, 0)
        End Function

        ''' <summary>
        ''' Initializes a FD capable PCAN Channel 
        ''' </summary>
        ''' <param name="Channel">The handle of a FD capable PCAN Channel</param>
        ''' <param name="BitrateFD">The speed for the communication (FD bit rate string)</param>
        ''' <remarks> See PCAN_BR_* values
        ''' Bit rate string must follow the following construction rules:
        ''' * parameter and values must be separated by '='
        ''' * Couples of Parameter/value must be separated by ','
        ''' * Following Parameter must be filled out: f_clock, data_brp, data_sjw, data_tseg1, data_tseg2,
        '''   nom_brp, nom_sjw, nom_tseg1, nom_tseg2.
        ''' * Following Parameters are optional (not used yet): data_ssp_offset, nom_sam</remarks>
        ''' <example>f_clock=80000000,nom_brp=10,nom_tseg1=5,nom_tseg2=2,nom_sjw=1,data_brp=4,data_tseg1=7,data_tseg2=2,data_sjw=1</example>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_InitializeFD")> _
        Public Shared Function InitializeFD( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByVal BitrateFD As TPCANBitrateFD) As TPCANStatus
        End Function

        ''' <summary>
        ''' Uninitializes one or all PCAN Channels initialized by CAN_Initialize
        ''' </summary>
        ''' <remarks>Giving the TPCANHandle value "PCAN_NONEBUS", 
        ''' uninitialize all initialized channels</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Uninitialize")> _
        Public Shared Function Uninitialize( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Resets the receive and transmit queues of the PCAN Channel
        ''' </summary>
        ''' <remarks>A reset of the CAN controller is not performed</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Reset")> _
        Public Shared Function Reset( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Gets the current status of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetStatus")> _
        Public Shared Function GetStatus( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle) As TPCANStatus
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>
        ''' <param name="TimestampBuffer">A TPCANTimestamp structure buffer to get
        ''' the reception time of the message</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Read")> _
        Public Shared Function Read( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg, _
            ByRef TimestampBuffer As TPCANTimestamp) As TPCANStatus
        End Function

        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Read")> _
        Private Shared Function Read( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg, _
            ByVal BufferPointer As IntPtr) As TPCANStatus
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg structure buffer to store the CAN message</param>        
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function Read( _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg) As TPCANStatus
            Return Read(Channel, MessageBuffer, IntPtr.Zero)
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a FD capable PCAN Channel 
        ''' </summary>
        ''' <param name="Channel">The handle of a FD capable PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsgFD structure buffer to store the CAN message</param>
        ''' <param name="TimestampBuffer">A TPCANTimestampFD buffer to get the
        ''' reception time of the message</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_ReadFD")> _
        Public Shared Function ReadFD( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsgFD, _
            ByRef TimestampBuffer As TPCANTimestampFD) As TPCANStatus
        End Function

        <DllImport("PCANBasic.dll", EntryPoint:="CAN_ReadFD")> _
        Private Shared Function ReadFD( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsgFD, _
            ByVal TimestampBuffer As IntPtr) As TPCANStatus
        End Function

        ''' <summary>
        ''' Reads a CAN message from the receive queue of a FD capable PCAN Channel 
        ''' </summary>
        ''' <param name="Channel">The handle of a FD capable PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsgFD structure buffer to store the CAN message</param>
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function ReadFD( _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsgFD) As TPCANStatus
            Return ReadFD(Channel, MessageBuffer, IntPtr.Zero)
        End Function

        ''' <summary>
        '''  Transmits a CAN message 
        ''' </summary>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsg buffer with the message to be sent</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_Write")> _
        Public Shared Function Write( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsg) As TPCANStatus
        End Function

        ''' <summary>
        ''' Transmits a CAN message over a FD capable PCAN Channel
        ''' </summary>
        ''' <param name="Channel">The handle of a FD capable PCAN Channel</param>
        ''' <param name="MessageBuffer">A TPCANMsgFD buffer with the message to be sent</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_WriteFD")> _
        Public Shared Function WriteFD( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByRef MessageBuffer As TPCANMsgFD) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures the reception filter
        ''' </summary>
        ''' <remarks>The message filter will be expanded with every call to 
        ''' this function. If it is desired to reset the filter, please use
        ''' the 'SetValue' function</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="FromID">The lowest CAN ID to be received</param>
        ''' <param name="ToID">The highest CAN ID to be received</param>
        ''' <param name="Mode">Message type, Standard (11-bit identifier) or
        ''' Extended (29-bit identifier)</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_FilterMessages")> _
        Public Shared Function FilterMessages( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            ByVal FromID As UInt32, _
            ByVal ToID As UInt32, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Mode As TPCANMode) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="StringBuffer">Buffer for the parameter value</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")> _
        Public Shared Function GetValue( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            ByVal StringBuffer As StringBuilder, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="NumericBuffer">Buffer for the parameter value</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")>
        Public Shared Function GetValue(
            <MarshalAs(UnmanagedType.U2)>
            ByVal Channel As TPCANHandle,
            <MarshalAs(UnmanagedType.U1)>
            ByVal Parameter As TPCANParameter,
            ByRef NumericBuffer As UInt32,
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="NumericBuffer">Buffer for the parameter value</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")>
        Public Shared Function GetValue(
            <MarshalAs(UnmanagedType.U2)>
            ByVal Channel As TPCANHandle,
            <MarshalAs(UnmanagedType.U1)>
            ByVal Parameter As TPCANParameter,
            ByRef NumericBuffer As UInt64,
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetValue")>
        Private Shared Function GetValue(
            <MarshalAs(UnmanagedType.U2)>
            ByVal Channel As TPCANHandle,
            <MarshalAs(UnmanagedType.U1)>
            ByVal Parameter As TPCANParameter,
            <MarshalAs(UnmanagedType.LPArray, SizeParamIndex:=3)>
            <[In], Out> ByVal ChannelsBuffer() As TPCANChannelInformation,
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Retrieves a PCAN Channel value
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to get</param>
        ''' <param name="ChannelsBuffer">Buffer for the parameter value</param>
        ''' <returns>A TPCANStatus error code</returns>
        Public Shared Function GetValue(
            ByVal Channel As TPCANHandle,
            ByVal Parameter As TPCANParameter,
            ByVal ChannelsBuffer() As TPCANChannelInformation) As TPCANStatus
            If (ChannelsBuffer Is Nothing) Then
                Return TPCANStatus.PCAN_ERROR_ILLPARAMVAL
            End If
            Return GetValue(Channel, Parameter, ChannelsBuffer, ChannelsBuffer.Length * Marshal.SizeOf(GetType(TPCANChannelInformation)))
        End Function

        ''' <summary>
        ''' Configures or sets a PCAN Channel value 
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to set</param>
        ''' <param name="NumericBuffer">Buffer with the value to be set</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_SetValue")>
        Public Shared Function SetValue(
            <MarshalAs(UnmanagedType.U2)>
            ByVal Channel As TPCANHandle,
            <MarshalAs(UnmanagedType.U1)>
            ByVal Parameter As TPCANParameter,
            ByRef NumericBuffer As UInt32,
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures or sets a PCAN Channel value 
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter">The TPCANParameter parameter to set</param>
        ''' <param name="NumericBuffer">Buffer with the value to be set</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_SetValue")>
        Public Shared Function SetValue(
            <MarshalAs(UnmanagedType.U2)>
            ByVal Channel As TPCANHandle,
            <MarshalAs(UnmanagedType.U1)>
            ByVal Parameter As TPCANParameter,
            ByRef NumericBuffer As UInt64,
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Configures or sets a PCAN Channel value 
        ''' </summary>
        ''' <remarks>Parameters can be present or not according with the kind 
        ''' of Hardware (PCAN Channel) being used. If a parameter is not available,
        ''' a PCAN_ERROR_ILLPARAMTYPE error will be returned</remarks>
        ''' <param name="Channel">The handle of a PCAN Channel</param>
        ''' <param name="Parameter"></param>
        ''' <param name="StringBuffer">Buffer with the value to be set</param>
        ''' <param name="BufferLength">Size in bytes of the buffer</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_SetValue")> _
        Public Shared Function SetValue( _
            <MarshalAs(UnmanagedType.U2)> _
            ByVal Channel As TPCANHandle, _
            <MarshalAs(UnmanagedType.U1)> _
            ByVal Parameter As TPCANParameter, _
            <MarshalAs(UnmanagedType.LPStr, SizeParamIndex:=3)> _
            ByVal StringBuffer As String, _
            ByVal BufferLength As UInt32) As TPCANStatus
        End Function

        ''' <summary>
        ''' Returns a descriptive text of a given TPCANStatus error 
        ''' code, in any desired language
        ''' </summary>
        ''' <remarks>The current languages available for translation are: 
        ''' Neutral (0x00), German (0x07), English (0x09), Spanish (0x0A),
        ''' Italian (0x10) and French (0x0C)</remarks>
        ''' <param name="anError">A TPCANStatus error code</param>
        ''' <param name="Language">Indicates a 'Primary language ID'</param>
        ''' <param name="StringBuffer">Buffer for the text (must be at least 256 in length)</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_GetErrorText")>
        Public Shared Function GetErrorText(
            <MarshalAs(UnmanagedType.U4)>
            ByVal anError As TPCANStatus,
            ByVal Language As UInt16,
            ByVal StringBuffer As StringBuilder) As TPCANStatus
        End Function

        ''' <summary>
        ''' Finds a PCAN-Basic channel that matches with the given parameters
        ''' </summary>
        ''' <param name="Parameters">A comma separated string contained pairs of 
        ''' parameter-name/value to be matched within a PCAN-Basic channel</param>
        ''' <param name="FoundChannel">Buffer for returning the PCAN-Basic channel, 
        ''' when found</param>
        ''' <returns>A TPCANStatus error code</returns>
        <DllImport("PCANBasic.dll", EntryPoint:="CAN_LookUpChannel")>
        Public Shared Function LookUpChannel(
            ByVal Parameters As String,
            ByRef FoundChannel As TPCANHandle) As TPCANStatus
        End Function
#End Region
    End Class
#End Region
End Namespace


