for f in $(find -name 'setup_deps.sh'); do
	echo "Found package setup file $f";
	chmod +x $f
	./$f
done