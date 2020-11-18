# ORIGINAL:
# Author: Ivan A-R <ivan@tuxotronic.org>
# Project page: http://tuxotronic.org/wiki/projects/stm32loader
#
# MODIFIED to add:
# * Input flushing and timing tuning in initChip() so that it works in spite of
#   large amounts of serial RX from the microcontroller
# * EEPROM saving option
# Author: Avik De <avikde@gmail.com>
# Project page: https://github.com/avikde/koduino
#
# This file is part of stm32loader.
#
# stm32loader is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 3, or (at your option) any later
# version.
#
# stm32loader is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with stm32loader; see the file COPYING3.  If not see
# <http://www.gnu.org/licenses/>.

from __future__ import print_function
from __future__ import unicode_literals

import sys, getopt
import serial
import time
import glob
import time
import tempfile
import os
import subprocess
import sys
import struct

if sys.version_info.major == 3:
    chr = lambda x: bytes([x])

try:
  import progressbar
  usepbar = 1
except:
  usepbar = 0

if usepbar:
	from progressbar import Percentage, ETA, Bar
	if sys.version_info.major == 3:
		# wrapper for difference between progressbar (python2)
		# and progressbar2 (python3)
		class ProgressBar(progressbar.ProgressBar):
			def __init__(self, **kwargs):
				if 'maxval' in kwargs:
					kwargs['max_value'] = kwargs.pop('maxval')
				super().__init__(**kwargs)
			@property
			def maxval(self):
				return self.max_value

	elif sys.version_info.major == 2:
		from progressbar import ProgressBar

# Verbose level
QUIET = 5

def mdebug(level, message):
  if QUIET >= level:
    print(message, file=sys.stderr)
    sys.stderr.flush()

# Takes chip IDs (obtained via Get ID command) to human-readable names
CHIP_ID_STRS = {
  0x410: 'STM32F1, performance, medium-density',
  0x411: 'STM32F2',
  0x412: 'STM32F1, performance, low-density',
  0x413: 'STM32F4',
  0x414: 'STM32F1, performance, high-density',
  0x416: 'STM32L1, performance, medium-density',
  0x418: 'STM32F1, connectivity',
  0x420: 'STM32F1, value, medium-density',
  0x428: 'STM32F1, value, high-density',
  0x430: 'STM32F1, performance, XL-density',
  0x432: 'F37x',
  0x439: 'F30x',
  0x421: 'F446',
  0x422: 'F303'
}

class CmdException(Exception):
  pass

class CommandInterface(object):
  def open(self, aport, abaudrate):
    self.sp = serial.Serial(
      port=aport,
      baudrate=abaudrate,     # baudrate
      bytesize=8,             # number of databits
      parity=serial.PARITY_EVEN,
      stopbits=1,
      xonxoff=0,              # enable software flow control
      rtscts=0,               # disable RTS/CTS flow control
      timeout=0.25             # set a timeout value, None for waiting forever
    )


  def _wait_for_ack(self, info="", timeout=0):
    stop = time.time() + timeout
    got = None
    while not got:
      got = self.sp.read(1)
      if time.time() > stop:
        break

    if not got:
      raise CmdException("No response to %s" % info)

    # wait for ask
    ask = ord(got)

    if ask == 0x79:
      # ACK
      return 1
    elif ask == 0x1F:
      # NACK
      raise CmdException("Chip replied with a NACK during %s" % info)

    # Unknown response
    raise CmdException("Unrecognized response 0x%x to %s" % (ask, info))

  def reset(self):
    self.sp.setDTR(True)
    time.sleep(0.1)
    self.sp.setDTR(False)
    time.sleep(0.1)

  def pulseRTS(self, delay):
    self.sp.setRTS(True)
    time.sleep(delay)
    self.sp.setRTS(False)
    time.sleep(0.1)

  def initChip(self, entry):
    # print(entry)
    if entry == 'dtr_rts':
      # Set boot
      self.sp.setRTS(False)
      time.sleep(0.1)
      self.reset()
    elif entry == 'mblc':
      # self.sp.setRTS(False)
      # self.sp.setDTR(True)
      # time.sleep(0.3)
      # self.sp.setDTR(False)

      self.sp.setRTS(True)
      self.sp.setDTR(False)
      time.sleep(0.1)
      self.sp.setRTS(False)
      self.sp.setDTR(True)
      time.sleep(0.3)
      self.sp.setDTR(False)
      time.sleep(0.05)
    elif entry == 'rts_trpl_inv':
      self.pulseRTS(0.15)

    # Be a bit more persistent when trying to initialise the chip
    stop = time.time() + 5

    # sys.stdout.write('Initing: ')

    while time.time() <= stop:
      if entry == 'dtr_rts':
        self.reset()

      self.sp.flushInput()
      #python2/3 with unicode literals
      self.sp.write(b'\x7f')

      got = self.sp.read()
      # if got:
        # sys.stdout.write(got)
      # The chip will ACK a sync the very first time and
      # NACK it every time afterwards
      #python2/3 with unicode_literals
      if got and got in b'\x79\x1f':
        # Synced up
        return

    raise CmdException('No response while trying to sync')

  def releaseChip(self, entry):
    if entry == 'dtr_rts':
      self.sp.setRTS(True)
      time.sleep(0.1)
      self.reset()
    elif entry == 'mblc':
      self.sp.setRTS(True)
      self.sp.setDTR(True)
      time.sleep(0.3)
      self.sp.setDTR(False)
    elif entry == 'rts_trpl_inv':
      self.pulseRTS(0.02)

  def cmdGeneric(self, cmd):
    self.sp.write(chr(cmd))
    self.sp.write(chr(cmd ^ 0xFF)) # Control byte
    return self._wait_for_ack(hex(cmd))

  def cmdGet(self):
    if self.cmdGeneric(0x00):
      mdebug(10, "*** Get command");
      len = ord(self.sp.read())
      version = ord(self.sp.read())
      #mdebug(10, "    Bootloader version: "+hex(version))
      dat = map(lambda c: hex(ord(c)), self.sp.read(len))
      mdebug(10, "    Available commands: "+str(dat))
      self._wait_for_ack("0x00 end")
      return version
    else:
      raise CmdException("Get (0x00) failed")

  def cmdGetVersion(self):
    if self.cmdGeneric(0x01):
      mdebug(10, "*** GetVersion command")
      version = ord(self.sp.read())
      self.sp.read(2)
      self._wait_for_ack("0x01 end")
      #mdebug(10, "    Bootloader version: "+hex(version))
      return version
    else:
      raise CmdException("GetVersion (0x01) failed")

  def cmdGetID(self):
    if self.cmdGeneric(0x02):
      mdebug(10, "*** GetID command")
      len = ord(self.sp.read())
      id = self.sp.read(len+1)
      self._wait_for_ack("0x02 end")
      return id
    else:
      raise CmdException("GetID (0x02) failed")


  def _encode_addr(self, addr):
    byte3 = (addr >> 0) & 0xFF
    byte2 = (addr >> 8) & 0xFF
    byte1 = (addr >> 16) & 0xFF
    byte0 = (addr >> 24) & 0xFF
    crc = byte0 ^ byte1 ^ byte2 ^ byte3
    return (chr(byte0) + chr(byte1) + chr(byte2) + chr(byte3) + chr(crc))


  def cmdReadMemory(self, addr, lng):
    assert(lng <= 256)
    if self.cmdGeneric(0x11):
      mdebug(10, "*** ReadMemory command")
      self.sp.write(self._encode_addr(addr))
      self._wait_for_ack("0x11 address failed")
      N = (lng - 1) & 0xFF
      crc = N ^ 0xFF
      self.sp.write(chr(N) + chr(crc))
      self._wait_for_ack("0x11 length failed")
      return map(lambda c: ord(c), self.sp.read(lng))
    else:
      raise CmdException("ReadMemory (0x11) failed")


  def cmdGo(self, addr):
    if self.cmdGeneric(0x21):
      mdebug(10, "*** Go command")
      self.sp.write(self._encode_addr(addr))
      self._wait_for_ack("0x21 go failed")
    else:
      raise CmdException("Go (0x21) failed")


  def cmdWriteMemory(self, addr, data):
    assert(len(data) <= 256)
    if self.cmdGeneric(0x31):
      mdebug(10, "*** Write memory command")
      self.sp.write(self._encode_addr(addr))
      self._wait_for_ack("0x31 address failed")
      #map(lambda c: hex(ord(c)), data)
      lng = (len(data)-1) & 0xFF
      mdebug(10, "    %s bytes to write" % [lng+1]);
      self.sp.write(chr(lng)) # len really
      crc = lng
      for c in data:
        crc = crc ^ c
        self.sp.write(chr(c))
      self.sp.write(chr(crc))
      self._wait_for_ack("0x31 programming failed")
      mdebug(10, "    Write memory done")
    else:
      raise CmdException("Write memory (0x31) failed")


  def cmdEraseMemory(self, sectors = None):
    if self.cmdGeneric(0x43):
      mdebug(10, "*** Erase memory command")
      if sectors is None:
        # Global erase
        self.sp.write(chr(0xFF))
        self.sp.write(chr(0x00))
      else:
        # Sectors erase
        self.sp.write(chr((len(sectors)-1) & 0xFF))
        crc = 0xFF
        for c in sectors:
          crc = crc ^ c
          self.sp.write(chr(c))
        self.sp.write(chr(crc))
      self._wait_for_ack("0x43 erasing failed")
      mdebug(10, "    Erase memory done")
    else:
      raise CmdException("Erase memory (0x43) failed")


  # TODO support for non-global mass erase (partially implemented by Garrett)
  GLOBAL_ERASE_TIMEOUT_SECONDS = 20   # This takes a while
  def cmdExtendedEraseMemory(self, sectors = None):
    if self.cmdGeneric(0x44):
      mdebug(10, "*** Extended erase memory command")
      if sectors is None:
        # Global mass erase
        #mdebug(5, "Global mass erase...")
        self.sp.write(chr(0xFF))
        self.sp.write(chr(0xFF))
        # Checksum
        self.sp.write(chr(0x00))
        self._wait_for_ack("0x44 global extended erase failed",
                   timeout=self.GLOBAL_ERASE_TIMEOUT_SECONDS)
        mdebug(10, "    Global extended erase memory done")
      else:
        # Erase sector numbers given as input
        mdebug(5, "Erasing sectors %s" % (sectors,))
        sectors = (len(sectors)-1,)+sectors
        # Convert sector numbers to 2 byte, MSB first
        sector_bytes = struct.pack('>%sH'%len(sectors), *sectors)
        crc = 0x00
        for i in range(len(sector_bytes)):
          # Calculate checksum
          crc = crc ^ ord(sector_bytes[i])
          self.sp.write(sector_bytes[i])
        self.sp.write(chr(crc))
        self._wait_for_ack("0x44 sector erase failed",
                    timeout=self.GLOBAL_ERASE_TIMEOUT_SECONDS)
        mdebug(10, "    Sector extended erase memory done")
    else:
      raise CmdException("Extended erase memory (0x44) failed")


  def cmdWriteProtect(self, sectors):
    if self.cmdGeneric(0x63):
      mdebug(10, "*** Write protect command")
      self.sp.write(chr((len(sectors)-1) & 0xFF))
      crc = 0xFF
      for c in sectors:
        crc = crc ^ c
        self.sp.write(chr(c))
      self.sp.write(chr(crc))
      self._wait_for_ack("0x63 write protect failed")
      mdebug(10, "    Write protect done")
    else:
      raise CmdException("Write Protect memory (0x63) failed")

  def cmdWriteUnprotect(self):
    if self.cmdGeneric(0x73):
      mdebug(10, "*** Write Unprotect command")
      self._wait_for_ack("0x73 write unprotect failed")
      self._wait_for_ack("0x73 write unprotect 2 failed")
      mdebug(10, "    Write Unprotect done")
    else:
      raise CmdException("Write Unprotect (0x73) failed")

  def cmdReadoutProtect(self):
    if self.cmdGeneric(0x82):
      mdebug(10, "*** Readout protect command")
      self._wait_for_ack("0x82 readout protect failed")
      self._wait_for_ack("0x82 readout protect 2 failed")
      mdebug(10, "    Read protect done")
    else:
      raise CmdException("Readout protect (0x82) failed")

  def cmdReadoutUnprotect(self):
    if self.cmdGeneric(0x92):
      mdebug(10, "*** Readout Unprotect command")
      self._wait_for_ack("0x92 readout unprotect failed")
      self._wait_for_ack("0x92 readout unprotect 2 failed")
      mdebug(10, "    Read Unprotect done")
    else:
      raise CmdException("Readout unprotect (0x92) failed")


# Complex commands section

  def readMemory(self, addr, lng):
    data = []
    if usepbar:
      widgets = ['Reading: ', Percentage(),', ', ETA(), ' ', Bar()]
      pbar = ProgressBar(widgets=widgets,maxval=lng, term_width=79).start()

    while lng > 256:
      if usepbar:
        pbar.update(pbar.maxval-lng)
        sys.stdout.flush()
        sys.stderr.flush()
      else:
        mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
      data = data + self.cmdReadMemory(addr, 256)
      addr = addr + 256
      lng = lng - 256
    if usepbar:
      pbar.update(pbar.maxval-lng)
      pbar.finish()
    else:
      mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
    data = data + self.cmdReadMemory(addr, lng)
    return data

  def writeMemory(self, addr, data):
    lng = len(data)

    #mdebug(5, "Writing %(lng)d bytes to start address 0x%(addr)X" %
    #     { 'lng': lng, 'addr': addr})

    if usepbar:
      widgets = ['', Percentage(),' ', ETA(), ' ', Bar()]
      pbar = ProgressBar(widgets=widgets, maxval=lng, term_width=79).start()

    offs = 0
    while lng > 256:
      if usepbar:
        pbar.update(pbar.maxval-lng)
        sys.stdout.flush()
        sys.stderr.flush()
      #else:
        #mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
      self.cmdWriteMemory(addr, data[offs:offs+256])
      offs = offs + 256
      addr = addr + 256
      lng = lng - 256
    if usepbar:
      pbar.update(pbar.maxval-lng)
      pbar.finish()
    #else:
    #  mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
    self.cmdWriteMemory(addr, data[offs:offs+lng] + ([0xFF] * (256-lng)) )


def usage():
  print("""Usage: %s [-hqVewvr] [-l length] [-p port] [-b baud] [-a addr] [file.bin]
  -h          This help
  -q          Quiet
  -V          Verbose
  -e          Erase
  -w          Write
  -v          Verify
  -r          Read
  -l length   Length of read
  -p port     Serial port (default: first USB-like port in /dev)
  -b baud     Baud speed (default: 57600)
  -a addr     Target address

  ./stm32loader.py -e -w -v example/main.bin

  """ % sys.argv[0])

def read(filename):
  """Read the file to be programmed and turn it into a binary"""
  with open(filename, 'rb') as f:
    bytes = f.read()

  if bytes.startswith(b'\x7FELF'):
    # Actually an ELF file.  Convert to binary
    handle, path = tempfile.mkstemp(suffix='.bin', prefix='stm32loader')

    try:
      os.close(handle)

      # Try a couple of options for objcopy
      for name in ['arm-none-eabi-objcopy', 'arm-linux-gnueabi-objcopy']:
        try:
          code = subprocess.call([name, '-Obinary', filename, path])

          if code == 0:
            return read(path)
        except OSError:
          pass
      else:
        raise Exception('Error %d while converting to a binary file' % code)
    finally:
      # Remove the temporary file
      os.unlink(path)
  else:
    if sys.version_info.major == 2:
        return [ord(x) for x in bytes]
    if sys.version_info.major == 3:
        return [x for x in bytes]

if __name__ == "__main__":

  # added "entry" property: can be dtr_rts (connected to NRST, BOOT0)
  #  or rts_trpl_inv (only RTS connected with triple inverter)

  conf = {
      'port': 'auto',
      'baud': 57600,
      'address': 0x08000000,
      'erase': 0,
      'write': 0,
      'verify': 0,
      'read': 0,
      'len': 1000,
      'fname':'',
      'eepstart': 0x08010000,
      'eeplen': 0,
      'entry': 'dtr_rts',
      'sector_erase': 0,
    }

# http://www.python.org/doc/2.5.2/lib/module-getopt.html

  try:
    opts, args = getopt.getopt(sys.argv[1:], "hqVewvrs:p:b:a:l:E:L:y:")
  except getopt.GetoptError as err:
    # print help information and exit:
    print(str(err)) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)

  # print(opts)

  for o, a in opts:
    if o == '-V':
      QUIET = 10
    elif o == '-q':
      QUIET = 0
    elif o == '-h':
      usage()
      sys.exit(0)
    elif o == '-e':
      conf['erase'] = 1
    elif o == '-s': # enables sector erase as opposed to mass erase
      conf['sector_erase'] = eval(a)
    elif o == '-w':
      conf['write'] = 1
    elif o == '-v':
      conf['verify'] = 1
    elif o == '-r':
      conf['read'] = 1
    elif o == '-p':
      conf['port'] = a
    elif o == '-b':
      conf['baud'] = eval(a)
    elif o == '-a':
      conf['address'] = eval(a)
    elif o == '-l':
      conf['len'] = eval(a)
    elif o == '-E':
      # add the base flash address
      conf['eepstart'] = 0x08000000+eval(a)
    elif o == '-L':
      conf['eeplen'] = eval(a)
    elif o == '-y':
      conf['entry'] = a
    else:
      assert False, "unhandled option"

  # Try and find the port automatically
  if conf['port'] == 'auto':
    ports = []

    # Get a list of all USB-like names in /dev
    for name in ['tty.usbserial', 'ttyUSB', 'ttyS']:
      ports.extend(glob.glob('/dev/%s*' % name))

    ports = sorted(ports)

    if ports:
      # Found something - take it
      conf['port'] = ports[0]

  # Find a port assuming a partial name was passed in, mostly this is to allow make to work
  # on different opertaing systems without specifying an upload port
  else:
    pass
    # # Get a list of ports that start with the passed in argument
    # ports = glob.glob('%s*' % conf['port'])
    # ports = sorted(ports)

    # # If the argument is actually in the port list, use the argument, otherwise grab
    # # the highest sorted port. E.g. if the port list is ttyUSB0, ttyUSB1, ttyUSB10 and the
    # # argument is ttyUSB1, then ports = ['ttyUSB1', 'ttyUSB10'] and ttyUSB1 will be used.
    # # If ttyUSB is the argument, ports = ['ttyUSB0', ttyUSB1', 'ttyUSB10'] and
    # # ttyUSB0 will be used
    # if conf['port'] not in ports:
    #   conf['port'] = ports[0]

  cmd = CommandInterface()
  cmd.open(conf['port'], conf['baud'])
  mdebug(10, "Open port %(port)s, baud %(baud)d" % {'port':conf['port'],
                            'baud':conf['baud']})

  try:
    try:
      cmd.initChip(conf['entry'])
    except CmdException:
      print("Can't init. Ensure BOOT0=1, BOOT1=0, and reset device")
      raise

    bootversion = cmd.cmdGet()

    #mdebug(0, "Bootloader version 0x%X" % bootversion)

    if bootversion < 20 or bootversion >= 100:
      raise Exception('Unreasonable bootloader version %d' % bootversion)
      sys.stdout.flush()

    chip_id = cmd.cmdGetID()
    assert len(chip_id) == 2, "Unreasonable chip id: %s" % repr(chip_id)
    if sys.version_info.major == 2:
        chip_id_num = (ord(chip_id[0]) << 8) | ord(chip_id[1])
    if sys.version_info.major == 3:
        chip_id_num = (chip_id[0] << 8) | chip_id[1]
    chip_id_str = CHIP_ID_STRS.get(chip_id_num, None)

    # mdebug(0, "Chip id 0x%x, %s" % (chip_id_num, chip_id_str))

    dualCore = False
    if (conf['write'] or conf['verify']):
      # Dual core two bins
      if len(args) > 1:
        data = [read(args[0]), read(args[1])]
        mdebug(5, "Reading dual core data from %s (big) and %s (little)" % (args[0], args[1]))
        dualCore = True
      else:
        mdebug(5, "Reading data from %s" % args[0])
        data = read(args[0])

    copiedEEPROM = False

    if conf['erase']:
      # Pre-3.0 bootloaders use the erase memory
      # command. Starting with 3.0, extended erase memory
      # replaced this command.
      if bootversion < 0x30:
        cmd.cmdEraseMemory()
      else:
        # Allow for quick erase instead of global mass erase (to save time)
        if conf['sector_erase'] and chip_id_num == 0x0413:
          # Currently hardcoded to only erase correct sectors for F405,
          # will need further improvements to work for other chips
          cmd.cmdExtendedEraseMemory(sectors = (0,3,4))
        else:
          if conf['sector_erase']:
            mdebug(0, 'Warning: sector erase currently does nothing for this chip version, defaulting to global mass erase')

          # First save EEPROM data
          if conf['eeplen'] > 0:
            eepdata = cmd.readMemory(conf['eepstart'], conf['eeplen'])
            copiedEEPROM = True

          cmd.cmdExtendedEraseMemory()

    #cmd.cmdWriteUnprotect()

    if conf['write']:
      if not usepbar:
        print("(Tip, run:    pip3 install progressbar2    to install progressbar2)");
      print("Flashing...")
      if chip_id_num == 0x0413:
        cmd.writeMemory(conf['address'], data[0x0000:512])
        cmd.writeMemory(conf['address']+0xC000, data[0xC000:])
      else:
        if dualCore:
          # Assume in the order m7, m4
          addresses = [0x08000000, 0x08100000]
          for i in range(2):
            cmd.writeMemory(addresses[i], data[i])
        else:
          cmd.writeMemory(conf['address'], data)

    if conf['verify'] and not dualCore:
      verify = cmd.readMemory(conf['address'], len(data))
      if(data == verify):
        print("Verification OK")
      else:
        print("Verification FAILED")
        print(str(len(data)) + ' vs ' + str(len(verify)))
        for i in xrange(0, len(data)):
          if data[i] != verify[i]:
            print(hex(i) + ': ' + hex(data[i]) + ' vs ' + hex(verify[i]))

    # Put back EEPROM data
    if copiedEEPROM:
      cmd.writeMemory(conf['eepstart'], eepdata)

    if not conf['write'] and conf['read']:
      rdata = cmd.readMemory(conf['address'], conf['len'])
      file(args[0], 'wb').write(''.join(map(chr,rdata)))

  finally:
    cmd.releaseChip(conf['entry'])

