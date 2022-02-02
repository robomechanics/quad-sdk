urdfreader - load models from (URDF Unified Robot Description Format) files
Copyright (c) 2020-2021 Felix Richter <judge@felixrichter.tech>

RBDL - URDF Reader Addon
========================
*This addon has been heavily updated from the original rbdl version!*

This addons is used to import URDF models into RBDL. It uses the [URDF_Parser Library](https://github.com/orb-hd/URDF_Parser) to parse the models. 

It is also possible to use the urdfdom distributed with ROS. To do this activate the ```RBDL_USE_ROS_URDF_LIBRARY``` cmake option during the compilation process!

Changelog
=========
04.12.20 - Replaced urdfdom with URDF_Parser library that has support for error handling and does not lead to crashes when loading models 
18.11.19 - Add support for urdfdom distributed with ROS
before - Version shipped by original rbdl

