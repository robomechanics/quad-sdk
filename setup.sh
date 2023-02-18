# Install all common dependencies
echo
echo "#########################"
echo "Install base dependencies"
echo "#########################"
echo
sudo apt install -y doxygen libeigen3-dev python3-catkin-tools python-pip
pip install cpplint

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
