# Install all common dependencies
echo
echo "#########################"
echo "Install base dependencies"
echo "#########################"
echo
sudo apt install doxygen

# Install all dependencies from package setup files
echo
echo "########################################################"
echo "Installing dependencies specified in package setup files"
echo "########################################################"
echo
for f in $(find -name 'setup_deps.sh'); do
	echo "Found package setup file $f";
	chmod +x $f
	./$f
done

# Install all rosdeps
echo
echo "#######################"
echo "Install all rosdeps"
echo "#######################"
echo
rosdep install --from-paths .. --ignore-src -r -y
