# UGVNavigationSystem
This is a UGV navigation system based on occupancy grid map in GPS-denied environments. We use open-source mapping algorithm _[Traversability Mapping](https://github.com/TixiaoShan/traversability_mapping)_ to build the environment map first and use _map_server_ ROS package to save the map to disk. we then back to the start point of mapping algorithm. based on the preliminary map model, once there is a goal point to send, the UGV will launch towards the target until it reachs the destination. We use _[LEGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)_ to locate the UGV, use _[ROS Navigation Stack](https://wiki.ros.org/navigation)_ to find optimal path and track the  generated trajectory and use _mqtt_ to communicate with remote terminals.

## 1. Prerequisite
### 1.1 Ubuntu and ROS
Ubuntu 64-bit 18.04 and ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
### 1.2 Create a catkin workspace
```sh
sudo apt install ros-melodic-catkin
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >>  ~/.bashrc
source ~/.bashrc

#check if success
echo $ROS_PACKAGE_PATH
```

