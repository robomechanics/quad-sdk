PCANBasicExample is a GUI sample presenting (almost) all features of PCAN-Basic API.
It uses Python interface to the Tcl/Tk GUI toolkit.


Installation:
-------------
 * Windows: no extra steps required.
	You can execute build.bat to build the example.

 * Linux: Tkinter (Python interface to Tcl/Tk GUI toolkit) is not installed by default.
	For Debian based distributions, you need to install the corresponding package: 
		* to install Tkinter for Pynthon 2.x:
			- apt-get install python-tk
		* to install Tkinter for Pynthon 3.x:
			- apt-get install python3-tk
	You can then use build.sh to build the example.


Usage:
------
TO run the sample call: 
	java peak.can.Application


Extra PCAN-Basic features:
--------------------------
Some features from PCAN-Basic API are not present in the GUI sample.
The following section adds short code samples for these.

 * LookUpChannel usage:
	// Crate New instance of PCANBasic
    PCANBasic pcanBasic = new PCANBasic();
    // JNI Initialization
    pcanBasic.initializeAPI();
	// ...		
	MutableTPCANHandle handle = new MutableTPCANHandle();
	TPCANStatus sts = pcanBasic.LookUpChannel(new StringBuffer("devicetype=pcan_usb"), handle);
	System.out.println("LookUpChannel: " + stsTmp + " => found handle is " + handle.getValue());