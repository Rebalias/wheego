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

```
pip install python-can==3.1.1 --user
```

* cantools version 29.7.0

```
pip install cantools==29.7.0 --user
```
  
It is possible that this version will fail to install, due to a bug where it falsely believes that it needs to install windows-curses, try installing it without dependencies, and then installing its dependencies one by one.

```
pip install --no-deps cantools==29.7.0 --user
pip install bitstruct
pip install diskcache
pip install textparser
```

#### Usage

To view LIDAR data, roslaunch viewer.launch. To use hector mapping, roslaunch mapper.launch. To use use full navigation, roslaunch testNav.launch, and the goal can be selected by placing a 2D Nav Goal in rviz. Each of the launch files has a sim argument set to True, change this to False in whichever one you launch for use with live data.

The primary pieces of code that we have developed are `map_handler.py`, `path_follow.py`, and `canBcastSteer.py`.

`map_handler.py`'s primary function is broadcasting a goal a few meters behind the one inputted into rviz, to account for the Navigation package's planning not accounting for the orientation of the goal, so that the car's path will cause it to arrive close to the intended orientation.

`path_follow.py`'s function is to perform the pure pursuit path following calculation [adapted from here](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf), and output the resulting steering angle to a ros topic.

`canBcastSteer.py` uses python-can and cantools to broadcast this steering angle and the can definitions established in `canSetup.kcd` to the CAN bus to control the power steering motor.

The role of path planning was previously filled by [Karl Kurzer's Path Planner](https://github.com/karlkurzer/path_planner), and roslaunching navigate.launch will attempt to launch it with the other components it if it is installed.


