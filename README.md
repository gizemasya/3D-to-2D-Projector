# 3D-to-2D-Projector
## ROS package that projects a 3D point and its unit vectors onto camera using KITTI rosbag
## 1. Prerequisites
This package has been tested with Ubuntu 20.04, ROS Noetic and OpenCV 4.
- C++11 or higher
- [ROS](http://wiki.ros.org/ROS/Installation)
- [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- OpenCV 4 (3 might be supported as well but not tested)
- [kitti2bag](https://github.com/tomas789/kitti2bag) (for KITTI rosbag)
## 2. Building
Clone the repository in your home:
```
    cd ~
    git clone https://github.com/gizemasya/3D-to-2D-Projector.git
```
Build the package with:
```
    cd 3D-to-2D-Projector/
    catkin_make
    source ~/3D-to-2D-Projector/devel/setup.bash
```
## 3. Usage
Firsly, run the node:
```
    rosrun projecting_frames_pkg Pinhole
```
In another terminal, start playing the KITTI rosbag:
```
    cd ~
    rosbag play kitti_2011_09_26_drive_0002_synced.bag
```
