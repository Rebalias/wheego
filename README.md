### KSU MTRE Fall 2019 Wheego Project

Kennesaw State University Mechatronics Engineering Senior Design Project Fall 2019, making the Wheego LiFe Car capable of autonomous parking lot navigation using the Velodyne VLP 16 LIDAR and a Jetson Xavier

Developed with Ubuntu Xenial and ROS Kinetic, tested with Ubuntu Bionic and ROS Melodic

#### Non-Standard Dependencies

##### ROS Packages

* [ROS Velodyne Package](https://wiki.ros.org/velodyne)
  * [Setup Guide for VLP-16 Model](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
* [ROS Pointcloud to Laserscan Package](https://wiki.ros.org/pointcloud_to_laserscan)
* [ROS Hector Mapping Package](https://wiki.ros.org/hector_mapping)
* [ROS Navigation Package](https://wiki.ros.org/navigation)

##### Python Libraries

* python-can verision 3.1.1

  `pip install python-can==3.1.1 --user`

* cantools version 29.7.0

  `pip install cantools==29.7.0 --user`

#### Usage

To view LIDAR data, roslaunch viewer.launch. To use hector mapping, roslaunch mapper.launch. To use use full navigation, roslaunch testNav.launch. Each of these files has a sim argument set to True. Change this to False for use with live data.
