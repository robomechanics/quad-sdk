# Installing Ghost Robotics build system for macOS and Linux

# Instructions:
# 	Open a terminal
#	Run the following:
# 		chmod +x install.sh
# 		./install.sh

echo 'Installing Ghost Robotics build system. This will take a few minutes.'

# Open working directory
mkdir -p ${HOME}/ghost_robotics
cd ${HOME}/ghost_robotics

# Install pip if not available
if [ -z "$(command -v pip)" ]; then
	echo 'Installing pip...'
	PIP='https://bootstrap.pypa.io/get-pip.py'
	curl -s -S -L ${PIP} -o get-pip.py
	sudo -H python get-pip.py 1> /dev/null 2> /dev/null
	rm get-pip.py
fi
echo 'pip installed.'

# Install pyserial if not available
if [ -z "$(pip show pyserial)" ]; then
	echo 'Installing pyserial...'
	sudo -H pip install pyserial 1> /dev/null 2> /dev/null
fi
echo 'pyserial installed.'

# Install progressbar if not available
if [ -z "$(pip show progressbar)" ]; then
	echo 'Installing progressbar...'
	sudo -H pip install progressbar 1> /dev/null 2> /dev/null
fi
echo 'progressbar installed.'

# Install curl if not available
if [ "$(uname -s)" == "Linux" ] && [ -z "$(command -v curl)" ]; then
	echo 'Installing curl...'
	sudo -H apt-get install curl
fi
echo "curl installed."

# Setup ARM Toolchain if not available
if [ ! -d ${HOME}/ghost_robotics/arm_toolchain ] && [ "$(uname -s)" == "Darwin" ]; then
	echo 'Downloading ARM Toolchain...'
	ARM_TOOLCHAIN='https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-mac.tar.bz2'
	curl -S -L ${ARM_TOOLCHAIN} -o get-arm_toolchain.tar.bz2 --progress-bar
	mkdir arm_toolchain
	tar xjf get-arm_toolchain.tar.bz2 -C arm_toolchain --strip-components 1
	chmod -R -w arm_toolchain
	rm get-arm_toolchain.tar.bz2
	echo -e '\nexport PATH=${HOME}/ghost_robotics/arm_toolchain/bin:${PATH}' >> ~/.bash_profile
elif [ ! -d ${HOME}/ghost_robotics/arm_toolchain ] && [ "$(uname -s)" == "Linux" ]; then
	echo 'Downloading ARM Toolchain...'
	ARM_TOOLCHAIN='https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2'
	curl -S -L ${ARM_TOOLCHAIN} -o get-arm_toolchain.tar.bz2 --progress-bar
	mkdir arm_toolchain
	tar xjf get-arm_toolchain.tar.bz2 -C arm_toolchain --strip-components 1
	chmod -R -w arm_toolchain
	rm get-arm_toolchain.tar.bz2
	echo -e '\nexport PATH=${HOME}/ghost_robotics/arm_toolchain/bin:${PATH}' >> ~/.bashrc
fi
echo 'ARM Toolchain installed.'

echo "You are now ready to upload to your robot."
echo "To get started, run:"
echo ""
echo "    cd examples/Stand"
echo "    make"
echo ""
