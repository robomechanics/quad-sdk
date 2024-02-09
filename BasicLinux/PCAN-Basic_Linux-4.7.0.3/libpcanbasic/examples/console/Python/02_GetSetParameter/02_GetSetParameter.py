## Needed Imports
import sys
from PCANBasic import *

class GetSetParameter():

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
        Create an object starts the programm
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

        print("Successfully initialized.")
        self.getInput("Press <Enter> to get/set parameter...")
        print("")
        self.RunSelectedCommands()
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
    def RunSelectedCommands(self):
        '''
        Runs all commands for get or set parameters
        '''
        ## Fill commands here 
        print("Fill \"RunSelectedCommands\"-function with parameter functions from \"Parameter commands\"-Region in the code.")

    # Parameter commands
    #region 
    def GetPCAN_DEVICE_ID(self):
        """
        Shows device identifier parameter
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_DEVICE_ID)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_DEVICE_ID: " + str(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_DEVICE_ID(self,intValue):
        """
        Sets device identifier parameter
        """
        if type(intValue) == int:
            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_DEVICE_ID,intValue)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_DEVICE_ID: " + str(intValue))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_DEVICE_ID wrong value type: " + str(type(intValue)))
            print("")

    def GetPCAN_ATTACHED_CHANNELS(self):
        """
        Shows all information about attached channels
        """
        stsResult = self.m_objPCANBasic.GetValue(PCAN_NONEBUS,PCAN_ATTACHED_CHANNELS_COUNT)

        if stsResult[0] == PCAN_ERROR_OK:
            stsResult = self.m_objPCANBasic.GetValue(PCAN_NONEBUS,PCAN_ATTACHED_CHANNELS)

            if stsResult[0] == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Get PCAN_ATTACHED_CHANNELS:")

                for currentChannelInformation in stsResult[1]:
                    print("---------------------------")
                    print("channel_handle:      " + self.ConvertToChannelHandle(currentChannelInformation.channel_handle))
                    print("device_type:         " + self.ConvertDeviceTypeToString(currentChannelInformation.device_type))
                    print("controller_number:   " + str(currentChannelInformation.controller_number))
                    print("device_features:     " + self.ConvertToChannelFeatures(currentChannelInformation.device_features))
                    print("device_name:         " + str(currentChannelInformation.device_name).replace("'","",2).replace("b","",1))
                    print("device_id:           " + str(currentChannelInformation.device_id))
                    print("channel_condition:   " + self.ConvertToChannelCondition(currentChannelInformation.channel_condition))
                print("")

        if stsResult[0] != PCAN_ERROR_OK:
            self.ShowStatus(stsResult[0])

    def GetPCAN_CHANNEL_CONDITION(self):
        """
        Shows the status of selected PCAN-Channel
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_CHANNEL_CONDITION)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_CHANNEL_CONDITION: " + self.ConvertToChannelCondition(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def GetPCAN_CHANNEL_IDENTIFYING(self):
        """
        Shows the status from the status LED of the USB devices
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_CHANNEL_IDENTIFYING)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_CHANNEL_IDENTIFYING: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_CHANNEL_IDENTIFYING(self,boolValue):
        """
        De/Activates the status LED of the USB devices

        Parameters:
            boolValue = True to turn on; False to turn off
        """
        if type(boolValue) == bool:
            if (boolValue):
                ciChannelIdentifying = PCAN_PARAMETER_ON
            else:
                ciChannelIdentifying = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_CHANNEL_IDENTIFYING,ciChannelIdentifying)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_CHANNEL_IDENTIFYING: " + self.ConvertToParameterOnOff(ciChannelIdentifying))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_CHANNEL_IDENTIFYING wrong value type: " + str(type(boolValue)))
            print("")

    def GetPCAN_CHANNEL_FEATURES(self):
        """
        Shows information about features
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_CHANNEL_FEATURES)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_CHANNEL_FEATURES: " + self.ConvertToChannelFeatures(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def GetPCAN_BITRATE_ADAPTING(self):
        """
        Shows the status from Bitrate-Adapting mode
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_BITRATE_ADAPTING)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_BITRATE_ADAPTING: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            ShowStatus(stsResult)

    def SetPCAN_BITRATE_ADAPTING(self,boolValue):
        """
        De/Activates the Bitrate-Adapting mode

        Parameters:
            boolValue = True to turn on; False to turn off
        """

        ## Note: SetPCAN_BITRATE_ADAPTING requires an uninitialized channel, 
        ##
        self.m_objPCANBasic.Uninitialize(PCAN_NONEBUS)

        if type(boolValue) == bool:
            if (boolValue):
                iBitrateAdapting = PCAN_PARAMETER_ON
            else:
                iBitrateAdapting = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_BITRATE_ADAPTING,iBitrateAdapting)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_BITRATE_ADAPTING: " + self.ConvertToParameterOnOff(iBitrateAdapting))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_BITRATE_ADAPTING wrong value type: " + str(type(boolValue)))
            print("")

        ## Initialization of the selected channel
        if self.IsFD:
            stsResult = self.m_objPCANBasic.InitializeFD(self.PcanHandle,self.BitrateFD)
        else:
            stsResult = self.m_objPCANBasic.Initialize(self.PcanHandle,self.Bitrate)

        if stsResult != PCAN_ERROR_OK:
            print("Error while re-initializing the channel.")
            self.ShowStatus(stsResult)

    def GetPCAN_ALLOW_STATUS_FRAMES(self):
        """
        Shows the status from the reception of status frames
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ALLOW_STATUS_FRAMES)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ALLOW_STATUS_FRAMES: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ALLOW_STATUS_FRAMES(self,boolValue):
        """
        De/Activates the reception of status frames

        Parameters:
            boolValue = True to turn on; False to turn off
        """
        if type(boolValue) == bool:
            if (boolValue):
                iAllowStatusFrames = PCAN_PARAMETER_ON
            else:
                iAllowStatusFrames = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ALLOW_STATUS_FRAMES,iAllowStatusFrames)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ALLOW_STATUS_FRAMES: " + self.ConvertToParameterOnOff(iAllowStatusFrames))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ALLOW_STATUS_FRAMES wrong value type: " + str(type(boolValue)))
            print("")

    def GetPCAN_ALLOW_RTR_FRAMES(self):
        """
        Shows the status from the reception of RTR frames
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ALLOW_RTR_FRAMES)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ALLOW_RTR_FRAMES: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ALLOW_RTR_FRAMES(self,boolValue):
        """
        De/Activates the reception of RTR frames

        Parameters:
            boolValue = True to turn on; False to turn off
        """
        if type(boolValue) == bool:
            if (boolValue):
                iAllowRTRFrames = PCAN_PARAMETER_ON
            else:
                iAllowRTRFrames = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ALLOW_RTR_FRAMES,iAllowRTRFrames)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ALLOW_RTR_FRAMES: " + self.ConvertToParameterOnOff(iAllowRTRFrames))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ALLOW_RTR_FRAMES wrong value type: " + str(type(boolValue)))
            print("")

    def GetPCAN_ALLOW_ERROR_FRAMES(self):
        """
        Shows the status from the reception of CAN error frames
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ALLOW_ERROR_FRAMES)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ALLOW_ERROR_FRAMES: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ALLOW_ERROR_FRAMES(self,boolValue):
        """
        De/Activates the reception of CAN error frames

        Parameters:
            boolValue = True to turn on; False to turn off
        """
        if type(boolValue) == bool:
            if (boolValue):
                iAllowErrorFrames = PCAN_PARAMETER_ON
            else:
                iAllowErrorFrames = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ALLOW_ERROR_FRAMES,iAllowErrorFrames)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ALLOW_ERROR_FRAMES: " + self.ConvertToParameterOnOff(iAllowErrorFrames))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ALLOW_ERROR_FRAMES wrong value type: " + str(type(boolValue)))
            print("")

    def GetPCAN_ALLOW_ECHO_FRAMES(self):
        """
        Shows the status from the reception of Echo frames
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ALLOW_ECHO_FRAMES)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ALLOW_ECHO_FRAMES: " + self.ConvertToParameterOnOff(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ALLOW_ECHO_FRAMES(self,boolValue):
        """
        De/Activates the reception of Echo frames

        Parameters:
            boolValue = True to turn on; False to turn off
        """
        if type(boolValue) == bool:
            if (boolValue):
                iAllowEchoFrames = PCAN_PARAMETER_ON
            else:
                iAllowEchoFrames = PCAN_PARAMETER_OFF

            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ALLOW_ECHO_FRAMES,iAllowEchoFrames)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ALLOW_ECHO_FRAMES: " + self.ConvertToParameterOnOff(iAllowEchoFrames))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ALLOW_ECHO_FRAMES wrong value type: " + str(type(boolValue)))
            print("")

    def GetPCAN_ACCEPTANCE_FILTER_11BIT(self):
        """
        Shows the reception filter with a specific 11-bit acceptance code and mask
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ACCEPTANCE_FILTER_11BIT)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ACCEPTANCE_FILTER_11BIT: " + hex(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ACCEPTANCE_FILTER_11BIT(self,iacceptancefilter11bit):
        """
        Sets the reception filter with a specific 11-bit acceptance code and mask

        Parameters:
            iacceptancefilter11bit = Acceptance code and mask
        """
        if type(iacceptancefilter11bit) == int:
            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ACCEPTANCE_FILTER_11BIT,iacceptancefilter11bit)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ACCEPTANCE_FILTER_11BIT: " + hex(iacceptancefilter11bit))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ACCEPTANCE_FILTER_11BIT wrong value type: " + str(type(iacceptancefilter11bit)))
            print("")

    def GetPCAN_ACCEPTANCE_FILTER_29BIT(self):
        """
        Shows the reception filter with a specific 29-bit acceptance code and mask
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_ACCEPTANCE_FILTER_29BIT)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_ACCEPTANCE_FILTER_29BIT: " + hex(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_ACCEPTANCE_FILTER_29BIT(self,iacceptancefilter29bit):
        """
        Sets the reception filter with a specific 29-bit acceptance code and mask

        Parameters:
            iacceptancefilter29bit = Acceptance code and mask
        """
        if type(iacceptancefilter29bit) == int:
            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_ACCEPTANCE_FILTER_29BIT,iacceptancefilter29bit)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_ACCEPTANCE_FILTER_29BIT: " + hex(iacceptancefilter29bit))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_ACCEPTANCE_FILTER_29BIT wrong value type: " + str(type(iacceptancefilter29bit)))
            print("")

    def GetPCAN_MESSAGE_FILTER(self):
        """
        Shows the status of the reception filter
        """
        stsResult = self.m_objPCANBasic.GetValue(self.PcanHandle,PCAN_MESSAGE_FILTER)

        if stsResult[0] == PCAN_ERROR_OK:
            print("-----------------------------------------------------------------------------------------")
            print("Get PCAN_MESSAGE_FILTER: " + self.ConvertToFilterOpenCloseCustom(stsResult[1]))
            print("")
        else:
            self.ShowStatus(stsResult)

    def SetPCAN_MESSAGE_FILTER(self,imessagefilter):
        """
        De/Activates the reception filter

        Parameters:
            imessagefilter = Configure reception filter
        """
        if type(imessagefilter) == int:
            stsResult = self.m_objPCANBasic.SetValue(self.PcanHandle,PCAN_MESSAGE_FILTER,imessagefilter)

            if stsResult == PCAN_ERROR_OK:
                print("-----------------------------------------------------------------------------------------")
                print("Set PCAN_MESSAGE_FILTER: " + self.ConvertToFilterOpenCloseCustom(imessagefilter))
                print("")
            else:
                self.ShowStatus(stsResult)
        else:
            print("-----------------------------------------------------------------------------------------")
            print("Warning! Set PCAN_MESSAGE_FILTER wrong value type: " + str(type(imessagefilter)))
            print("")
    #endregion
    #endregion

    # Help-Functions
    #region
    def ShowConfigurationHelp(self):
        """
        Shows/prints the configurable parameters for this sample and information about them
        """
        print("=========================================================================================")
        print("|                        PCAN-Basic GetSetParameter Example                              |")
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
        Shows/prints the configured paramters
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
        print("=========================================================================================")#

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

    def ConvertToChannelHandle(self,value):
        switcher = {PCAN_USBBUS1.value:"PCAN_USBBUS1",
                    PCAN_USBBUS2.value:"PCAN_USBBUS2",
                    PCAN_USBBUS3.value:"PCAN_USBBUS3",
                    PCAN_USBBUS4.value:"PCAN_USBBUS4",
                    PCAN_USBBUS5.value:"PCAN_USBBUS5",
                    PCAN_USBBUS6.value:"PCAN_USBBUS6",
                    PCAN_USBBUS7.value:"PCAN_USBBUS7",
                    PCAN_USBBUS8.value:"PCAN_USBBUS8",
                    PCAN_USBBUS9.value:"PCAN_USBBUS9",
                    PCAN_USBBUS10.value:"PCAN_USBBUS10",
                    PCAN_USBBUS11.value:"PCAN_USBBUS11",
                    PCAN_USBBUS12.value:"PCAN_USBBUS12",
                    PCAN_USBBUS13.value:"PCAN_USBBUS13",
                    PCAN_USBBUS14.value:"PCAN_USBBUS14",
                    PCAN_USBBUS15.value:"PCAN_USBBUS15",
                    PCAN_USBBUS16.value:"PCAN_USBBUS16",
                    
                    PCAN_LANBUS1.value:"PCAN_LANBUS1",
                    PCAN_LANBUS2.value:"PCAN_LANBUS2",
                    PCAN_LANBUS3.value:"PCAN_LANBUS3",
                    PCAN_LANBUS4.value:"PCAN_LANBUS4",
                    PCAN_LANBUS5.value:"PCAN_LANBUS5",
                    PCAN_LANBUS6.value:"PCAN_LANBUS6",
                    PCAN_LANBUS7.value:"PCAN_LANBUS7",
                    PCAN_LANBUS8.value:"PCAN_LANBUS8",
                    PCAN_LANBUS9.value:"PCAN_LANBUS9",
                    PCAN_LANBUS10.value:"PCAN_LANBUS10",
                    PCAN_LANBUS11.value:"PCAN_LANBUS11",
                    PCAN_LANBUS12.value:"PCAN_LANBUS12",
                    PCAN_LANBUS13.value:"PCAN_LANBUS13",
                    PCAN_LANBUS14.value:"PCAN_LANBUS14",
                    PCAN_LANBUS15.value:"PCAN_LANBUS15",
                    PCAN_LANBUS16.value:"PCAN_LANBUS16",
                    
                    PCAN_PCIBUS1.value:"PCAN_PCIBUS1",
                    PCAN_PCIBUS2.value:"PCAN_PCIBUS2",
                    PCAN_PCIBUS3.value:"PCAN_PCIBUS3",
                    PCAN_PCIBUS4.value:"PCAN_PCIBUS4",
                    PCAN_PCIBUS5.value:"PCAN_PCIBUS5",
                    PCAN_PCIBUS6.value:"PCAN_PCIBUS6",
                    PCAN_PCIBUS7.value:"PCAN_PCIBUS7",
                    PCAN_PCIBUS8.value:"PCAN_PCIBUS8",
                    PCAN_PCIBUS9.value:"PCAN_PCIBUS9",
                    PCAN_PCIBUS10.value:"PCAN_PCIBUS10",
                    PCAN_PCIBUS11.value:"PCAN_PCIBUS11",
                    PCAN_PCIBUS12.value:"PCAN_PCIBUS12",
                    PCAN_PCIBUS13.value:"PCAN_PCIBUS13",
                    PCAN_PCIBUS14.value:"PCAN_PCIBUS14",
                    PCAN_PCIBUS15.value:"PCAN_PCIBUS15",
                    PCAN_PCIBUS16.value:"PCAN_PCIBUS16",}

        if value in switcher:
            return switcher[value]
        else:
            return "Handle unknown: " + str(value)

        
    def ConvertDeviceTypeToString(self, devicetype):
        """
        Convert BYTE value to readable string value
        """
        switcher = {PCAN_NONE.value:"PCAN_NONE",
                    PCAN_PEAKCAN.value:"PCAN_PEAKCAN",
                    PCAN_DNG.value:"PCAN_DNG",
                    PCAN_PCI.value:"PCAN_PCI",
                    PCAN_USB.value:"PCAN_USB",
                    PCAN_VIRTUAL.value:"PCAN_VIRTUAL",
                    PCAN_LAN.value:"PCAN_LAN",}

        if devicetype in switcher:
            return switcher[devicetype]
        else:
            return "Status unknown: " + str(devicetype)

    def ConvertToParameterOnOff(self,value):
        """
        Convert uint value to readable string value
        """
        switcher = {PCAN_PARAMETER_OFF:"PCAN_PARAMETER_OFF",
                    PCAN_PARAMETER_ON:"PCAN_PARAMETER_ON"}

        if value in switcher:
            return switcher[value]
        else:
            return "Status unknown: " + str(value)

    def ConvertToChannelFeatures(self,value):
        """
        Convert uint value to readable string value
        """
        sFeatures = ""

        if (value & FEATURE_FD_CAPABLE) == FEATURE_FD_CAPABLE:
            sFeatures += "FEATURE_FD_CAPABLE"
        if ((value & FEATURE_DELAY_CAPABLE) == FEATURE_DELAY_CAPABLE):
                if (sFeatures != ""):
                    sFeatures += ", FEATURE_DELAY_CAPABLE"
                else:
                    sFeatures += "FEATURE_DELAY_CAPABLE"
        if ((value & FEATURE_IO_CAPABLE) == FEATURE_IO_CAPABLE):
            if (sFeatures != ""):
                sFeatures += ", FEATURE_IO_CAPABLE"
            else:
                sFeatures += "FEATURE_IO_CAPABLE"
        return sFeatures;

    def ConvertToChannelCondition(self,value):
        """
        Convert uint value to readable string value
        """
        switcher = {PCAN_CHANNEL_UNAVAILABLE:"PCAN_CHANNEL_UNAVAILABLE",
                    PCAN_CHANNEL_AVAILABLE:"PCAN_CHANNEL_AVAILABLE",
                    PCAN_CHANNEL_OCCUPIED:"PCAN_CHANNEL_OCCUPIED",
                    PCAN_CHANNEL_PCANVIEW:"PCAN_CHANNEL_PCANVIEW"}

        if value in switcher:
            return switcher[value]
        else:
            return "Status unknown: " + str(value)

    def ConvertToFilterOpenCloseCustom(self,value):
        """
        Convert uint value to readable string value
        """
        switcher = {PCAN_FILTER_CLOSE:"PCAN_FILTER_CLOSE",
                    PCAN_FILTER_OPEN:"PCAN_FILTER_OPEN",
                    PCAN_FILTER_CUSTOM:"PCAN_FILTER_CUSTOM"}

        if value in switcher:
            return switcher[value]
        else:
            return "Status unknown: " + str(value)

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

## Starts the program
GetSetParameter()

