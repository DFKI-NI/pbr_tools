This package contains a gazebo world of the ICO, MOELK and CIC office environments.

The urdf folder contains urdf files of the pbr_robot_lab, as well as simulated rgb_camera urdfs connected to the CIC walls to simulate the insect hotel demo cameras.

Warning: The nodes in the nodes folder are not ported to ROS2 and are not working for ROS2!
The nodes folder has the gazebo_unpauser script used during startup of Gazebo in ROS1, which fixes some issues during the spawning of the objects and setting the initial arm pose. Other than that it contains a script to get object poses of objects in Gazebo automatically in XML format.

The Media/models folder has the walls of the pbr_robot_lab and pbr_cic environments.
