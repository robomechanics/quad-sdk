#!/bin/bash

# Usage: upload.sh [binfile] [port]
# Will autodetect port on Mac, Linux, Win otherwise

PORT=

SDK_DIR=../..

# Binary file to upload is first command line arg
BINFILE="MAVLinkSpeedTest_mb_SPIRIT_0x12.bin"
if [ "$1" != "" ]; then
	BINFILE=$1
fi

# Port override is second command line arg
if [ "$2" != "" ]; then
	PORT=$2
fi

# Get platform
unameOut="$(uname -s)"
case "${unameOut}" in
    Linux*)     machine=Linux;;
    Darwin*)    machine=Mac;;
    CYGWIN*)    machine=Win;;
    MINGW*)     machine=Win;;
    *)          machine=Win;;
esac

# Auto select port if no port provided
if [ "$PORT" == "" ]; then
    if [ "$machine" == "Linux" ]; then
        PORT=`ls /dev/ttyUSB*`
    elif [ "$machine" == "Mac" ]; then
        PORT=`ls /dev/tty.usbserial*`
    elif [ "$machine" == "Win" ]; then
        PORT=COM4
    fi
fi
if [ "$PORT" == "" ]; then
	PORT="NO_ROBOT_FOUND"
fi

# Upload mainboard bin
CMDLINE="python $SDK_DIR/tools/stm32loader.py -p ${PORT} -b 230400 -y mblc -E 524288 -L 0 -ew ${BINFILE}"
#echo $CMDLINE
echo $PORT
# Try up to five times
for i in 1 2 3 4 5; do $CMDLINE && break || sleep 5; done

