# Wheego

Kennesaw State University Mechatronics Engineering Senior Design Project Fall 2019, making the Wheego LiFe Car capable of autonomous parking lot navigation using the Velodyne VLP 16 LIDAR and a Jetson Xavier

Developed on Ubuntu Xenial and ROS Kinetic, should be compatible with Ubuntu Bionic and ROS Melodic

## Non-Standard Dependencies

### ROS Packages

* [Velodyne](https://wiki.ros.org/velodyne)
  * [Setup Guide](https://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
* [Pointcloud to Laserscan](https://wiki.ros.org/pointcloud_to_laserscan)
* [Hector Mapping](https://wiki.ros.org/hector_mapping)
* [Karl Kurzer's Hybrid A\* Path Planner](https://github.com/karlkurzer/path_planner)

### Python Libraries

* python-can verision 3.1.1

  `pip install python-can==3.1.1 --user`

* cantools version 29.7.0

  `pip install cantools==29.7.0 --user`

## Usage

To view LIDAR data, roslaunch viewer.launch. To view prerecorded LIDAR data such as a rosbag, alter the sim argument in viewer.launch to be true instead of false

To use hector mapping, roslaunch mapper.launch. This will use the sim argument set in viewer.launch.

To use use full navigation, roslaunch navigate.launch. Again, this will use the sim argument set in viewer.launch.
