'pcaninfo' lists all known PCAN devices and outputs information for each.

-----------------------------------------------
Exemple: 
--------
PCAN driver version: 8.0.6

Found 2 PCAN devices
  * pcanusb1: (/sys/class/usbmisc/pcanusb1)
 	- file: /dev/pcanusb1
  	- dev: "180:1"
  	- minor: 1
  	-----------------
	- type: "usbfd"
	- hwtype: "USB FD" (18)
	- devid: 0x00
	- serial_number: 0
	- ctrl_number: 0
	-----------------
	- clock: 80 MHz
	- bitrate: 500 kBit/s
	- btr0btr1: 0x1c
	- dbitrate: 2 MBit/s
	-----------------
	- bus_state: Closed / Unknown (0)
	- bus_load: 0%
	- rx_error_counter: 0
	- tx_error_counter: 0
	-----------------
	- irqs: 0
	- status: 0
	- errors: 0
 	- read: 0
 	- write: 0
  	-----------------
	
  * pcanusb2: (/sys/class/usbmisc/pcanusb2)
	- file: /dev/pcanusb2
	- dev: "180:2"
	- minor: 2
	-----------------
	- type: "usbpro"
	- hwtype: "USB Pro" (13)
	- devid: 0x0b
	- serial_number: -1
	- ctrl_number: 0
	-----------------
	- clock: 56 MHz
	- bitrate: 500 kBit/s
	- btr0btr1: 0x1c
	-----------------
	- bus_state: Closed / Unknown (0)
	- bus_load: 0%
	-----------------
	- irqs: 0
	- status: 0
	- errors: 0
	- read: 0
	- write: 0
	-----------------

-----------------------------------------------
