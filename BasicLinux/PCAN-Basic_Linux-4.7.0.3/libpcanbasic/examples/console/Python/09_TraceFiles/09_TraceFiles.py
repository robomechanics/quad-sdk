## Needed Imports
from PCANBasic import *
import sys
import time
import threading

class TraceFiles():

    # Defines
    #region

    # Sets the PCANHandle (Hardware Channel)
    PcanHandle = PCAN_USBBUS1

    # Sets the desired connection mode (CAN = false / CAN-FD = true)
    IsFD = False

    # Sets the bitrate for normal CAN devices
    Bitrate = PCAN_BAUD_500K

    # Sets the bitrate for CAN FD devices. 
    # Example - Bitrate Nom: 1Mbit/s Data: 2Mbit/s:
    #   "f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1"
    BitrateFD = b'f_clock_mhz=20, nom_brp=5, nom_tseg1=2, nom_tseg2=1, nom_sjw=1, data_brp=2, data_tseg1=3, data_tseg2=1, data_sjw=1'

    # Sets if trace continue after reaching maximum size for the first file
    TraceFileSingle = True
    
    # Set if date will be add to filename
    TraceFileDate = True

    # Set if time will be add to filename
    TraceFileTime = True

    # Set if existing tracefile overwrites when a new trace session is started
    TraceFileOverwrite = False

    # Set if the column "Data Length" should be used instead of the column "Data Length Code"
    TraceFileDataLength = False

    # Sets the size (megabyte) of an tracefile 
    # Example - 100 = 100 megabyte
    # Range between 1 and 100
    TraceFileSize = 2

    # Sets a fully-qualified and valid path to an existing directory. In order to use the default path 
    # (calling process path) an empty string must be set.
    TracePath = b''
    #endregion
    
    # Members
    #region

    # Shows if DLL was found
    m_DLLFound = False

    #endregion

    def __init__(self):
        '''
        Create an object starts programm
        '''
        self.ShowConfigurationHelp() ## Shows information about this sample
        self.ShowCurrentConfiguration() ## Shows the current parameters configuration

        ## Checks if PCANBasic.dll is available, if not, the program terminates
        try:
            self.m_objPCANBasic = PCANBasic()
            self.m_DLLFound = True
        except :
            print("Unable to find the library: PCANBasic.dll !")
            self.getInput("Press <Enter> to quit...")
            self.m_DLLFound = False
            return
        
        ## Initialization of the selected channel
        if self.IsFD:
            stsResult = self.m_objPCANBasic.InitializeFD(self.PcanHandle,self.BitrateFD)
        else:
            stsResult = self.m_objPCANBasic.Initialize(self.PcanHandle,self.Bitrate)

        if stsResult != PCAN_ERROR_OK:
            print("Can not initialize. Please check the defines in the code.")
            self.ShowStatus(stsResult)
            print("")
            self.getInput("Press <Enter> to quit...")
            return

        ## Trace messages...
        print("Successfully initialized.")
        self.getInput("Press <Enter> to start tracing...")
        if self.ConfigureTrace():
            if self.StartTrace():
                self.m_objThread = threading.Thread(target = self.ThreadExecute, args = ())
                self.m_ThreadRun = True
                self.m_objThread.start()
                print("Messages are being traced.")
                self.getInput("Press <Enter> to stop trace...")
                self.StopTrace()
                self.m_ThreadRun = False
                self.m_objThread.join() 
        print("")
        self.getInput("Press <Enter> to exit...")

    def __del__(self):
        if self.m_DLLFound:
            self.m_objPCANBasic.Uninitialize(PCAN_NONEBUS)

    def getInput(self, msg="Press <Enter> to continue...", default=""):
        res = default
        if sys.version_info[0] >= 3:
            res = input(msg + " ")
        else:
            res = raw_input(msg + " ")
        if len(res) == 0:
            res = default
        return res

    # Main-Functions
    #region
    def ThreadExecute(self):
        '''
        Thread function for reading messages
        '''
        while self.m_ThreadRun:
            ## time.sleep(1) ## Use Sleep to reduce the CPU load   
            self.ReadMessages()

    def ReadMessages(self):
        """
        Reads PCAN-Basic messages
        """
        stsResult = [PCAN_ERROR_OK]
        ## We read at least one time the queue looking for messages. If a message is found, we look again trying to find more.
        ## If the queue is empty or an error occurr, we get out from the while statement.
        while stsResult[0] != PCAN_ERROR_QRCVEMPTY:
            if self.IsFD:
                stsResult = self.m_objPCANBasic.ReadFD(self.PcanHandle)
            else:
                stsResult = self.m_objPCANBasic.Read(self.PcanHandle)
            if stsResult[0] != PCAN_ERROR_OK and stsResult[0] != PCAN_ERROR_QRCVEMPTY:
                self.ShowStatus(stsResult[0])
                break

    def StopTrace(self):
        """
        Deactivates the tracing process
        """
        ## We stop the tracing by setting the parameter.
        stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_TRACE_STATUS, PCAN_PARAMETER_OFF)
        if stsResult != PCAN_ERROR_OK:
            self.ShowStatus(stsResult)
        else:
            self.ThreadRun = False

    def ConfigureTrace(self):
        """
        Configures the way how trace files are formatted
        """
        stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_TRACE_LOCATION, self.TracePath) ## Sets path to store files
        if stsResult == PCAN_ERROR_OK:

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_TRACE_SIZE, self.TraceFileSize) ## Sets the maximum size of file
            if (stsResult == PCAN_ERROR_OK):
                
                if (self.TraceFileSingle):
                    config = TRACE_FILE_SINGLE ## Creats one file
                else:
                    config = TRACE_FILE_SEGMENTED ## Creats more files

                ## Overwrites existing tracefile
                if self.TraceFileOverwrite:
                    config = config | TRACE_FILE_OVERWRITE

                ## Uses Data Length instead of Data Length Code
                if self.TraceFileDataLength:
                    config = config | TRACE_FILE_DATA_LENGTH
                
                ## Adds date to tracefilename
                if self.TraceFileDate:
                    config = config | TRACE_FILE_DATE

                 ## Adds time to tracefilename
                if self.TraceFileTime:
                    config = config | TRACE_FILE_TIME

                stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_TRACE_CONFIGURE, config)
                if (stsResult == PCAN_ERROR_OK):
                    return True
        self.ShowStatus(stsResult)
        return False

    def StartTrace(self):
        """
        Activates the tracing process
        """
        ## We activate the tracing by setting the parameter.
        stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle, PCAN_TRACE_STATUS, PCAN_PARAMETER_ON)
        if stsResult != PCAN_ERROR_OK:
            self.ShowStatus(stsResult)
            return False
        self.ThreadRun = True
        return True
    #endregion

    # Help-Functions
    #region
    def ShowConfigurationHelp(self):
        """
        Shows/prints the configurable parameters for this sample and information about them
        """
        print("=========================================================================================")
        print("|                           PCAN-Basic Trace-Files Example                               |")
        print("=========================================================================================")
        print("Following parameters are to be adjusted before launching, according to the hardware used |")
        print("                                                                                         |")
        print("* PcanHandle: Numeric value that represents the handle of the PCAN-Basic channel to use. |")
        print("              See 'PCAN-Handle Definitions' within the documentation                     |")
        print("* IsFD: Boolean value that indicates the communication mode, CAN (false) or CAN-FD (true)|")
        print("* Bitrate: Numeric value that represents the BTR0/BR1 bitrate value to be used for CAN   |")
        print("           communication                                                                 |")
        print("* BitrateFD: String value that represents the nominal/data bitrate value to be used for  |")
        print("             CAN-FD communication                                                        |")
        print("* TraceFileSingle: Boolean value that indicates if tracing ends after one file (true) or |")
        print("                   continues                                                             |")
        print("* TraceFileDate: Boolean value that indicates if the date will be added to filename      |")
        print("* TraceFileTime: Boolean value that indicates if the time will be added to filename      |")
        print("* TraceFileOverwrite: Boolean value that indicates if existing tracefiles should be      |")
        print("                      overwritten                                                        |")
        print("* TraceFileDataLength: Boolean value that indicates if the column 'Data Length' is used  |")
        print("                       instead of the column 'Data Length Code'                          |")
        print("* TraceFileSize: Numeric value that represents the size of a tracefile in meagabytes     |")
        print("* TracePath: String value that represents a valid path to an existing directory          |")
        print("=========================================================================================")
        print("")

    def ShowCurrentConfiguration(self):
        """
        Shows/prints the configured paramters
        """
        print("Parameter values used")
        print("----------------------")
        print("* PCANHandle: " + self.FormatChannelName(self.PcanHandle))
        print("* IsFD: " + str(self.IsFD))
        print("* Bitrate: " + self.ConvertBitrateToString(self.Bitrate))
        print("* BitrateFD: " + self.ConvertBytesToString(self.BitrateFD))
        print("* TraceFileSingle: " + str(self.TraceFileSingle))
        print("* TraceFileDate: " + str(self.TraceFileDate))
        print("* TraceFileTime: " + str(self.TraceFileTime))
        print("* TraceFileOverwrite: " + str(self.TraceFileOverwrite))
        print("* TraceFileDataLength: " + str(self.TraceFileDataLength))
        print("* TraceFileSize: " + str(self.TraceFileSize) + " MB")
        if self.TracePath == b'':
            print("* TracePath: (calling application path)")
        else:
            print("* TracePath: " + self.ConvertBytesToString(self.TracePath))
        print("")

    def ShowStatus(self,status):
        """
        Shows formatted status

        Parameters:
            status = Will be formatted
        """
        print("=========================================================================================")
        print(self.GetFormattedError(status))
        print("=========================================================================================")
    
    def FormatChannelName(self, handle, isFD=False):
        """
        Gets the formated text for a PCAN-Basic channel handle

        Parameters:
            handle = PCAN-Basic Handle to format
            isFD = If the channel is FD capable

        Returns:
            The formatted text for a channel
        """
        handleValue = handle.value
        if handleValue < 0x100:
            devDevice = TPCANDevice(handleValue >> 4)
            byChannel = handleValue & 0xF
        else:
            devDevice = TPCANDevice(handleValue >> 8)
            byChannel = handleValue & 0xFF

        if isFD:
           return ('%s:FD %s (%.2Xh)' % (self.GetDeviceName(devDevice.value), byChannel, handleValue))
        else:
           return ('%s %s (%.2Xh)' % (self.GetDeviceName(devDevice.value), byChannel, handleValue))

    def GetFormattedError(self, error):
        """
        Help Function used to get an error as text

        Parameters:
            error = Error code to be translated

        Returns:
            A text with the translated error
        """
        ## Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
        ## If it fails, a text describing the current error is returned.
        stsReturn = self.m_objPCANBasic.GetErrorText(error,0x09)
        if stsReturn[0] != PCAN_ERROR_OK:
            return "An error occurred. Error-code's text ({0:X}h) couldn't be retrieved".format(error)
        else:
            message = str(stsReturn[1])
            return message.replace("'","",2).replace("b","",1)
      
    def GetDeviceName(self, handle):
        """
        Gets the name of a PCAN device

        Parameters:
            handle = PCAN-Basic Handle for getting the name

        Returns:
            The name of the handle
        """
        switcher = {
            PCAN_NONEBUS.value: "PCAN_NONEBUS",
            PCAN_PEAKCAN.value: "PCAN_PEAKCAN",
            PCAN_DNG.value: "PCAN_DNG",
            PCAN_PCI.value: "PCAN_PCI",
            PCAN_USB.value: "PCAN_USB",
            PCAN_VIRTUAL.value: "PCAN_VIRTUAL",
            PCAN_LAN.value: "PCAN_LAN"
        }

        return switcher.get(handle,"UNKNOWN")   

    def ConvertBitrateToString(self, bitrate):
        """
        Convert bitrate c_short value to readable string

        Parameters:
            bitrate = Bitrate to be converted

        Returns:
            A text with the converted bitrate
        """
        m_BAUDRATES = {PCAN_BAUD_1M.value:'1 MBit/sec', PCAN_BAUD_800K.value:'800 kBit/sec', PCAN_BAUD_500K.value:'500 kBit/sec', PCAN_BAUD_250K.value:'250 kBit/sec',
                       PCAN_BAUD_125K.value:'125 kBit/sec', PCAN_BAUD_100K.value:'100 kBit/sec', PCAN_BAUD_95K.value:'95,238 kBit/sec', PCAN_BAUD_83K.value:'83,333 kBit/sec',
                       PCAN_BAUD_50K.value:'50 kBit/sec', PCAN_BAUD_47K.value:'47,619 kBit/sec', PCAN_BAUD_33K.value:'33,333 kBit/sec', PCAN_BAUD_20K.value:'20 kBit/sec',
                       PCAN_BAUD_10K.value:'10 kBit/sec', PCAN_BAUD_5K.value:'5 kBit/sec'}
        return m_BAUDRATES[bitrate.value]

    def ConvertBytesToString(self, bytes):
        """
        Convert bytes value to string

        Parameters:
            bytes = Bytes to be converted

        Returns:
            Converted bytes value as string
        """
        return str(bytes).replace("'","",2).replace("b","",1)
    #endregion

## Starts the sample
TraceFiles()
