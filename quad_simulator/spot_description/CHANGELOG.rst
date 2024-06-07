^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package spot_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2023-08-06)
------------------

1.0.0 (2023-04-07)
------------------
* update maintainer
* adressed layout review comments
* moved urdf
* updated urdf to include arm
* add correct joint for front rail, should be directly between first screws. Rear rail is based on that
* remove inorder flag
* Use the velodyne_description package for the actual sensor mesh, use the cage as a separate entity. Add an accessories.launch file to automatically bring up the velodyne if needed
* Include the license file in the individual ROS packages
* Added optional pack and standard velodyne mount
* fix joint limits typo
* add install instructions
* Added base_link (rep-0105)
* Updated URDF to include mount points, and deps for install
* Made the driver not automatically claim a body lease and e stop.  Allows you to monitor without having control.  Disconnect doesn't exit very gracefully.  Need to wait on sitting success
* Added SPOT_URDF_EXTRAS environment variable for extending the robot
* Initial commit
* Contributors: Chris Iverach-Brereton, Dave Niewinski, Juan Miguel Jimeno, Mario Gini, Michal Staniaszek, Wolf Vollprecht, maubrunn
