#!/bin/sh
CLASS_DIR="./classes"
SRC_DIR="./src"

if [ ! -d "$CLASS_DIR" ]; then
	mkdir "$CLASS_DIR"
fi
javac -d "$CLASS_DIR" $(find "$SRC_DIR"/* | grep .java)
