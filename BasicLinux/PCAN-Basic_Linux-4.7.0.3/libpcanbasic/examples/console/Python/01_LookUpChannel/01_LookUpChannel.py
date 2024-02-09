## Needed Imports
import sys
from PCANBasic import *

class LookUpChannel():

    # Defines
    #region

    # Sets a TPCANDevice value. The input can be numeric, in hexadecimal or decimal format, or as string denoting 
    # a TPCANDevice value name.
    DeviceType = b"PCAN_USB"

    # Sets value in range of a double. The input can be hexadecimal or decimal format.
    DeviceID = b""

    # Sets a zero-based index value in range of a double. The input can be hexadecimal or decimal format.
    ControllerNumber = b""

    # Sets a valid Internet Protocol address 
    IPAddress = b""

    #endregion

    # Members
    #region

    # Shows if DLL was found
    m_DLLFound = False

    #endregion

    def __init__(self):
        """
        Create an object starts programm
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

        self.getInput("Press <Enter> to start searching...")
        print("")

        self.sParameters = b''
        if self.DeviceType != b'':
            self.sParameters += LOOKUP_DEVICE_TYPE + b'=' + self.DeviceType
        if self.DeviceID != b'':
            if self.sParameters != b'':
                self.sParameters += b', '
            self.sParameters += LOOKUP_DEVICE_ID + b'=' + self.DeviceID
        if self.ControllerNumber != b'':
            if self.sParameters != b'':
                self.sParameters += b', '
            self.sParameters += LOOKUP_CONTROLLER_NUMBER+ b'=' + self.ControllerNumber
        if self.IPAddress != b'':
            if self.sParameters != b'':
                self.sParameters += b', '
            self.sParameters += LOOKUP_IP_ADDRESS + b'=' + self.IPAddress

        stsResult = self.m_objPCANBasic.LookUpChannel(self.sParameters)

        if stsResult[0] == PCAN_ERROR_OK:
            handle = stsResult[1]
            if handle != PCAN_NONEBUS:
                stsResult = self.m_objPCANBasic.GetValue(handle, PCAN_CHANNEL_FEATURES)

                if stsResult[0] == PCAN_ERROR_OK:
                    print("The channel handle " + self.FormatChannelName(handle, (stsResult[1] & FEATURE_FD_CAPABLE) == FEATURE_FD_CAPABLE) + " was found")
                else:
                    print("There was an issue retrieving supported channel features")
            else:
                print("A handle for these lookup-criteria was not found")
        
        if stsResult[0] != PCAN_ERROR_OK:
            print("There was an error looking up the device, are any hardware channels attached?")
            self.ShowStatus(stsResult[0])

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

    # Help-Functions
    #region
    def ShowConfigurationHelp(self):
        """
        Shows/prints the configurable parameters for this sample and information about them
        """
        print("=========================================================================================")
        print("|                        PCAN-Basic LookUpChannel Example                                |")
        print("=========================================================================================")
        print("Following parameters are to be adjusted before launching, according to the hardware used |")
        print("                                                                                         |")
        print("* DeviceType: Numeric value that represents a TPCANDevice                                |")
        print("* DeviceID: Numeric value that represents the device identifier                          |")
        print("* ControllerNumber: Numeric value that represents controller number                      |")
        print("* IPAddress: String value that represents a valid Internet Protocol address              |")
        print("                                                                                         |")
        print("For more information see 'LookUp Parameter Definition' within the documentation          |")
        print("=========================================================================================")
        print("")

    def ShowCurrentConfiguration(self):
        """
        Shows/prints the configured paramters
        """
        print("Parameter values used")
        print("----------------------")
        print('* DeviceType: ' + self.ConvertBytesToString(self.DeviceType))
        print("* DeviceID: " + self.ConvertBytesToString(self.DeviceID))
        print("* ControllerNumber: " + self.ConvertBytesToString(self.ControllerNumber))
        print("* IPAddress: " + self.ConvertBytesToString (self.IPAddress))
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
           return ('%s: FD %s (%.2Xh)' % (self.GetDeviceName(devDevice.value), byChannel, handleValue))
        else:
           return ('%s: %s (%.2Xh)' % (self.GetDeviceName(devDevice.value), byChannel, handleValue))

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
LookUpChannel()