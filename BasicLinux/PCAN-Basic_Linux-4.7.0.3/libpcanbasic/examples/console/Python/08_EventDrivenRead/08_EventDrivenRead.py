## Needed Imports
from PCANBasic import *
import sys
import threading

IS_WINDOWS = platform.system() == 'Windows'

# Try to support interoperable  events
try:
  if IS_WINDOWS: 
    import win32api
    import win32event
  else:
    import errno
    import os
    import select
    __LIBC_OBJ = cdll.LoadLibrary("libc.so.6")
    def eventfd(init_val, flags):
      return __LIBC_OBJ.eventfd(init_val, flags)
  EVENT_SUPPORTED = True
except Exception as ex:
  EVENT_SUPPORTED = False
  print("Failed to support interoperable-event, exception=(" + str(ex) + ")")  

class EventDrivenRead():

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

    #endregion
    
    # Members
    #region

    # Shows if DLL was found
    m_DLLFound = False

    #endregion

    def __init__(self):
        """
        Create an object starts the program
        """
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

        if IS_WINDOWS and not EVENT_SUPPORTED:
            print("The Win32 Library ('Python Win32 Extensions') is not installed.")
            return

        ## Reading messages...
        print("Successfully initialized.")
        (stsResult,self.m_ReadAbort) = self.InitializeEvent()
        self.m_ReadThread = threading.Thread(None, self.CANReadThreadFunc)
        self.m_ReadThread.start()
        print("Started reading messages...")
        print("")
        self.getInput("Press <Enter> to stop reading...")
        self.m_ThreadRun = False
        self.SetEvent(self.m_ReadAbort)
        self.m_ReadThread.join()
        self.UninitializeEvent(self.m_ReadAbort)
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
    def CANReadThreadFunc(self):
        """
        Thread-Function used for reading PCAN-Basic messages
        """
        try:        
            self.m_ThreadRun = True
        
            ## Configures the Receive-Event. 
            ##
            (stsResult, self.m_ReceiveEvent) = self.InitializeEvent(Channel=self.PcanHandle)

            if stsResult != PCAN_ERROR_OK:
                print ("Error: " + self.GetFormatedError(stsResult))                
            else:
                while self.m_ThreadRun:
                    stsResult = self.WaitForEvent(self.m_ReceiveEvent, waitTimeout=10000, abortEvent=self.m_ReadAbort)
                    if stsResult == PCAN_ERROR_OK:
                        self.ReadMessages()
            
                ## Resets the Event-handle configuration
                ##            
                stsResult = self.UninitializeEvent(self.m_ReceiveEvent, Channel=self.PcanHandle)
                self.m_ReceiveEvent = None
        except:
            print ("Error occurred while processing CAN data")   

    def ReadMessages(self):
        """
        Function for reading PCAN-Basic messages
        """
        stsResult = PCAN_ERROR_OK
        ## We read at least one time the queue looking for messages. If a message is found, we look again trying to 
        ## find more. If the queue is empty or an error occur, we get out from the do-while statement.
        while (not (stsResult & PCAN_ERROR_QRCVEMPTY)):
            if self.IsFD:
                stsResult = self.ReadMessageFD()
            else:
                stsResult = self.ReadMessage()
            if stsResult != PCAN_ERROR_OK and stsResult != PCAN_ERROR_QRCVEMPTY:
                self.ShowStatus(stsResult)
                return
         
    def ReadMessage(self):
        """
        Function for reading CAN messages on normal CAN devices

        Returns:
            A TPCANStatus error code
        """
        ## We execute the "Read" function of the PCANBasic   
        stsResult = self.m_objPCANBasic.Read(self.PcanHandle)

        if stsResult[0] == PCAN_ERROR_OK:
            ## We show the received message
            self.ProcessMessageCan(stsResult[1],stsResult[2])
            
        return stsResult[0]

    def ReadMessageFD(self):
        """
        Function for reading messages on CAN-FD devices

        Returns:
            A TPCANStatus error code
        """
        ## We execute the "Read" function of the PCANBasic    
        stsResult = self.m_objPCANBasic.ReadFD(self.PcanHandle)

        if stsResult[0] == PCAN_ERROR_OK:
            ## We show the received message
            self.ProcessMessageCanFd(stsResult[1],stsResult[2])
            
        return stsResult[0]

    def ProcessMessageCan(self,msg,itstimestamp):
         """
         Processes a received CAN message
         
         Parameters:
             msg = The received PCAN-Basic CAN message
             itstimestamp = Timestamp of the message as TPCANTimestamp structure
         """
         microsTimeStamp = itstimestamp.micros + 1000 * itstimestamp.millis + 0x100000000 * 1000 * itstimestamp.millis_overflow
         
         print("Type: " + self.GetTypeString(msg.MSGTYPE))
         print("ID: " + self.GetIdString(msg.ID, msg.MSGTYPE))
         print("Length: " + str(msg.LEN))
         print("Time: " + self.GetTimeString(microsTimeStamp))
         print("Data: " + self.GetDataString(msg.DATA,msg.MSGTYPE))
         print("----------------------------------------------------------")

    def ProcessMessageCanFd(self,msg,itstimestamp):
        """
        Processes a received CAN-FD message

        Parameters:
            msg = The received PCAN-Basic CAN-FD message
            itstimestamp = Timestamp of the message as microseconds (ulong)
        """
        print("Type: " + self.GetTypeString(msg.MSGTYPE))
        print("ID: " + self.GetIdString(msg.ID, msg.MSGTYPE))
        print("Length: " + str(self.GetLengthFromDLC(msg.DLC)))
        print("Time: " + self.GetTimeString(itstimestamp))
        print("Data: " + self.GetDataString(msg.DATA,msg.MSGTYPE))
        print("----------------------------------------------------------")
    #endregion

    # Help-Functions
    #region
    def ShowConfigurationHelp(self):
        """
        Shows/prints the configurable parameters for this sample and information about them
        """
        print("=========================================================================================")
        print("|                        PCAN-Basic EventDrivenRead Example                              |")
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
        print("=========================================================================================")
        print("")

    def ShowCurrentConfiguration(self):
        """
        Shows/prints the configured parameters
        """
        print("Parameter values used")
        print("----------------------")
        print("* PCANHandle: " + self.FormatChannelName(self.PcanHandle,self.IsFD))
        print("* IsFD: " + str(self.IsFD))
        print("* Bitrate: " + self.ConvertBitrateToString(self.Bitrate))
        print("* BitrateFD: " + self.ConvertBytesToString(self.BitrateFD))
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
    
    def FormatChannelName(self, handle, isFD):
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

    def GetLengthFromDLC(dlc):
        """
        Gets the data length of a CAN message

        Parameters:
            dlc = Data length code of a CAN message

        Returns:
            Data length as integer represented by the given DLC code
        """
        if dlc == 9:
            return 12
        elif dlc == 10:
            return 16
        elif dlc == 11:
            return 20
        elif dlc == 12:
            return 24
        elif dlc == 13:
            return 32
        elif dlc == 14:
            return 48
        elif dlc == 15:
            return 64
        
        return dlc

    def GetIdString(self, id, msgtype):
        """
        Gets the string representation of the ID of a CAN message

        Parameters:
            id = Id to be parsed
            msgtype = Type flags of the message the Id belong

        Returns:
            Hexadecimal representation of the ID of a CAN message
        """
        if (msgtype & PCAN_MESSAGE_EXTENDED.value) == PCAN_MESSAGE_EXTENDED.value:
            return '%.8Xh' %id
        else:
            return '%.3Xh' %id

    def GetTimeString(self, time):
        """
        Gets the string representation of the timestamp of a CAN message, in milliseconds

        Parameters:
            time = Timestamp in microseconds

        Returns:
            String representing the timestamp in milliseconds
        """
        fTime = time / 1000.0
        return '%.1f' %fTime

    def GetTypeString(self, msgtype):  
        """
        Gets the string representation of the type of a CAN message

        Parameters:
            msgtype = Type of a CAN message

        Returns:
            The type of the CAN message as string
        """
        if (msgtype & PCAN_MESSAGE_STATUS.value) == PCAN_MESSAGE_STATUS.value:
            return 'STATUS'
        
        if (msgtype & PCAN_MESSAGE_ERRFRAME.value) == PCAN_MESSAGE_ERRFRAME.value:
            return 'ERROR'        
        
        if (msgtype & PCAN_MESSAGE_EXTENDED.value) == PCAN_MESSAGE_EXTENDED.value:
            strTemp = 'EXT'
        else:
            strTemp = 'STD'

        if (msgtype & PCAN_MESSAGE_RTR.value) == PCAN_MESSAGE_RTR.value:
            strTemp += '/RTR'
        else:
            if (msgtype > PCAN_MESSAGE_EXTENDED.value):
                strTemp += ' ['
                if (msgtype & PCAN_MESSAGE_FD.value) == PCAN_MESSAGE_FD.value:
                    strTemp += ' FD'
                if (msgtype & PCAN_MESSAGE_BRS.value) == PCAN_MESSAGE_BRS.value:                    
                    strTemp += ' BRS'
                if (msgtype & PCAN_MESSAGE_ESI.value) == PCAN_MESSAGE_ESI.value:
                    strTemp += ' ESI'
                strTemp += ' ]'
                
        return strTemp

    def GetDataString(self, data, msgtype):
        """
        Gets the data of a CAN message as a string

        Parameters:
            data = Array of bytes containing the data to parse
            msgtype = Type flags of the message the data belong

        Returns:
            A string with hexadecimal formatted data bytes of a CAN message
        """
        if (msgtype & PCAN_MESSAGE_RTR.value) == PCAN_MESSAGE_RTR.value:
            return "Remote Request"
        else:
            strTemp = b""
            for x in data:
                strTemp += b'%.2X ' % x
            return str(strTemp).replace("'","",2).replace("b","",1)

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
            PCAN_LAN.value: "PCAN_LAN"}

        return switcher.get(handle,"UNKNOWN")   

    def ConvertBitrateToString(self, bitrate):
        """
        Convert bitrate c_short value to readable string

        Parameters:
            bitrate = Bitrate to be converted

        Returns:
            A text with the converted bitrate
        """
        m_BAUDRATES = {PCAN_BAUD_1M.value:'1 MBit/sec', 
                       PCAN_BAUD_800K.value:'800 kBit/sec', 
                       PCAN_BAUD_500K.value:'500 kBit/sec', 
                       PCAN_BAUD_250K.value:'250 kBit/sec',
                       PCAN_BAUD_125K.value:'125 kBit/sec', 
                       PCAN_BAUD_100K.value:'100 kBit/sec', 
                       PCAN_BAUD_95K.value:'95,238 kBit/sec', 
                       PCAN_BAUD_83K.value:'83,333 kBit/sec',
                       PCAN_BAUD_50K.value:'50 kBit/sec', 
                       PCAN_BAUD_47K.value:'47,619 kBit/sec', 
                       PCAN_BAUD_33K.value:'33,333 kBit/sec', 
                       PCAN_BAUD_20K.value:'20 kBit/sec',
                       PCAN_BAUD_10K.value:'10 kBit/sec', 
                       PCAN_BAUD_5K.value:'5 kBit/sec'}

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

    # Help-Functions - event handling
    #region
    def InitializeEvent(self, Channel:TPCANHandle=None):
        """
            Creates a PCAN-Basic event to be used with SetEvent and WaitForEvent functions. 

        Remarks:

            The return value of this method is a 2-tuple, where 
            the first value is the result (TPCANStatus) of the method and
            the second one an OS-dependent object representing the event

        Example - creates an event to abort a call to WaitForEvent
            (sts,abortEvent) = objPCANBasic.InitializeEvent()
            # [...] WaitForEvent(..) is called in another thread with a reference to abortEvent
            objPCANBasic.SetEvent(abortEvent)
            objPCANBasic.UninitializeEvent(abortEvent)

        Example - creates an event to wait indefinitely for channel’s messages with WaitForEvent
            (sts, receiveEvent) = objPCANBasic.InitializeEvent(pcanHandle)
            stsResult = objPCANBasic.WaitForEvent(self.m_ReceiveEvent, waitTimeout=None, abortEvent=abortEvent)
            # [...] handle status and read messages
            objPCANBasic.UninitializeEvent(receiveEvent)

        Parameters:
            Channel      : An optional TPCANHandle representing a PCAN Channel. 
                           If set, the resulting event will be notified of receive event on that channel.

        Returns:
            A tuple with 2 values: (TPCANStatus error code, Event)
        """
        if EVENT_SUPPORTED:
            stsResult = PCAN_ERROR_OK
            if IS_WINDOWS:
                event = win32event.CreateEvent(None, 0, 0, None)
                if event and Channel != None:
                    stsResult = self.m_objPCANBasic.SetValue(Channel, PCAN_RECEIVE_EVENT, event)
            else:
                if Channel != None:
                    (stsResult, event) = self.m_objPCANBasic.GetValue(Channel, PCAN_RECEIVE_EVENT)
                else:
                    event = eventfd(0, os.EFD_NONBLOCK)
        else:
            stsResult = PCAN_ERROR_NODRIVER
            event = None

        return stsResult, event

    def UninitializeEvent(self, event, Channel:TPCANHandle=None):
        """
            Deletes a PCAN-Basic event. 

        Parameters:
            Channel      : An optional TPCANHandle representing a PCAN Channel. 
                           If set, the channel removes receive-notification to that event.

        Returns:
            TPCANStatus error code
        """
        if not EVENT_SUPPORTED:
            return PCAN_ERROR_NODRIVER

        stsResult = PCAN_ERROR_OK
        if IS_WINDOWS:
            if event:
                win32api.CloseHandle(event)

            if Channel != None:
                stsResult = self.m_objPCANBasic.SetValue(Channel, PCAN_RECEIVE_EVENT, 0)
        else:
            if Channel == None:
                os.close(event)

        return stsResult
     
    def SetEvent(self, event):
        """
            Sets a PCAN-Basic event waking up WaitForEvent related function. 

        Parameters:
            event      : The PCANBasic (OS-dependent) event to set.

        Returns:
            TPCANStatus error code
        """
        if not EVENT_SUPPORTED:
            return PCAN_ERROR_NODRIVER

        stsResult = PCAN_ERROR_OK
        if IS_WINDOWS:
            win32event.SetEvent(event)
        else:
            dummy = c_uint64(1)
            ret = -1
            loop = True
            while loop:
                ret = os.write(event, bytearray(dummy))
                loop = ret == -1 and ctypes.get_errno() == errno.EAGAIN

        return stsResult

    def WaitForEvent(self, receiveEvent, waitTimeout:int=None, abortEvent=None):
        """
            Waits until a PCANBasic event occurs (message received or abort) or a timeout is reached.
                  
        Parameters:
            Channel      : A TPCANHandle representing a PCAN Channel.
            waitTimeout  : If set, the maximum number of milliseconds to wait for.
            abortEvent   : If set, an OS-dependent event to abort the wait (see InitializeEvent).

        Returns:
            A TPCANStatus error code
        """
        if not EVENT_SUPPORTED:
            return PCAN_ERROR_NODRIVER

        if IS_WINDOWS:
            timeout = waitTimeout if waitTimeout != None else win32event.INFINITE
            events = [ receiveEvent ]
            
            if abortEvent != None:
                events.append(abortEvent)

            res = win32event.WaitForMultipleObjects(events, False, waitTimeout) 

            if res == win32event.WAIT_OBJECT_0 + 0:
                stsResult = PCAN_ERROR_OK
            elif res == win32event.WAIT_OBJECT_0 + 1:
                stsResult = PCAN_ERROR_QRCVEMPTY
            elif res == win32event.WAIT_TIMEOUT:
                stsResult = PCAN_ERROR_QRCVEMPTY
            else:
                stsResult = PCAN_ERROR_RESOURCE

        else:
            timeout = waitTimeout/1000 if waitTimeout != None else None
            events = [ receiveEvent ]
            
            if abortEvent != None:
                events.append(abortEvent)

            readable, _, _ = select.select(events, [], [], waitTimeout/1000)

            if len(readable) > 0:
                if receiveEvent in readable:
                    stsResult = PCAN_ERROR_OK
                elif abortEvent in readable:
                    stsResult = PCAN_ERROR_QRCVEMPTY
                else:
                    stsResult = PCAN_ERROR_RESOURCE
            else:
                stsResult = PCAN_ERROR_QRCVEMPTY

        return stsResult
    #endregion
    
## Starts the program
EventDrivenRead()
