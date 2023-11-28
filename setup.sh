# Install all common dependencies
echo
echo "#########################"
echo "Install base dependencies"
echo "#########################"
echo
<<<<<<< HEAD
sudo apt install -y doxygen libeigen3-dev python3-catkin-tools python3-pip
pip install cpplint
=======
<<<<<<< HEAD
sudo apt install -y doxygen libeigen3-dev python3-catkin-tools python-pip
pip install cpplint
=======
sudo apt install -y doxygen libeigen3-dev
sudo apt-get install python3-catkin-tools
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

# Install all dependencies from package setup files
echo
echo "########################################################"
echo "Installing dependencies specified in package setup files"
echo "########################################################"
echo
CWD=$(pwd)
for f in $(find -name 'setup_deps.sh'); do
	echo "Found package setup file $f";
	BASE=$(dirname $f)
	cd "$BASE"
	chmod +x setup_deps.sh
	./setup_deps.sh
	cd "$CWD"
done

# Install all rosdeps
echo
echo "#######################"
echo "Install all rosdeps"
echo "#######################"
echo
rosdep install --from-paths .. --ignore-src -r -y --rosdistro noetic
