# UGVNavigationSystem
This is a UGV navigation system based on occupancy grid map in GPS-denied environments. We use open-source mapping algorithm _[Traversability Mapping](https://github.com/TixiaoShan/traversability_mapping)_ to build the environment map first and use _map_server_ ROS package to save the map to disk. we then back to the start point of mapping algorithm. based on the preliminary map model, once there is a goal point to send, the UGV will launch towards the target until it reachs the destination. We use _[LEGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)_ to locate the UGV, use _[ROS Navigation Stack](https://wiki.ros.org/navigation)_ to find optimal path and track the  generated trajectory and use _mqtt_ to communicate with remote terminals.

## 1. Prerequisite
### 1.1 first do
Ubuntu and ROS
Ubuntu 64-bit 18.04 and ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
Create a catkin workspace
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
ROS Package dependecies
```sh
sudo apt install ros-melodic-navigation
```
### 1.2 Receive lidar data
- install the dependencies in _[rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)_
- download via git
```sh
cd ~/catkin_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
```
- compile with ROS catkin tools following _[rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk)_
- modify lidar_type: RSHELIOS in config.yaml
- run
`roslaunch rslidar_sdk start.launch`
- open rviz, you can see the pointcloud in frame /rslidar

### 1.3 LeGO-LOAM
- download via git
```sh
cd ~/catkin_ws/src
git clone https://github.com/RobustFieldAutonomyLab/LeGO-LOAM.git
```
- modify global parameters in _utility.h_
```c++
extern const string pointCloudTopic = "/rslidar_points";
//RSHELIOS
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1800;
extern const float ang_res_x = 0.2;
extern const float ang_res_y = 1.0;
extern const float ang_bottom = 16;
extern const int groundScanInd = 7;
```
### 1.4 Mapping
- download via git
```sh
cd ~/catkin_ws/src
git clone https://github.com/TixiaoShan/traversability_mapping.git
```
- modify global parameters in _utility.h_
```c++
// RS-Helios vertical and horizontal points per scan 
extern const int N_SCAN = 32;
extern const int Horizon_SCAN = 1800;
// Robot Params
extern const float robotRadius = 0.6;
extern const float sensorHeight = 0.9;
// length of the local occupancy grid map (meter)
extern const int localMapLength = 300; 
```
- run the launch file
`roslaunch traversability_mapping online.launch`
- save grid map topic:=/occupancy_map_local name:=center_map2
`rosrun map_server map_saver  -f center_map2 map:=/occupancy_map_local`

### 1.5 Path planning and Control
- download this repository via git
```sh
cd ~/catkin_ws/src
git clone https://github.com/WillenChung/UGVNavigationSystem.git
```
- modify parameters in move_base.launch&car_ctr.launch
```xml
<!-- move_base.launch -->

<!-- load map config file -->
<arg name="map" value="new_map_c.yaml" />
```
```xml
<!-- car_ctr.launch -->

<!-- configure car speed 0-1000 -->
<arg name="car_speed" value="400"/>
<!-- configure PID params -->
<arg name="P" value="10.0"/>
<arg name="I" value="0.12"/>
<arg name="D" value="0"/>
<arg name="IMax" value="50"/>
<arg name="PIDMax" value="150"/>
<arg name="I_Threshold" value="15"/>
<arg name="I_DeadArea" value="1"/>
<!-- configure following path topic -->
<arg name="follow_path_set" value="/move_base/NavfnROS/plan"/>
```
### 1.6 Communication with remote

## 2. Run
### 2.1 Mapping
```sh
cd $PATH_mapping.sh
bash mapping.sh
```
1. publish msg:sensor_msgs/PointCloud2 in topic:/rslidar_points
`roslaunch rslidar_sdk start.launch`
2. run traversability_mapping
`roslaunch traversability_mapping online.launch`
3. move car around the environment
4. save occupancy grid map
`cd /home/willen/test_ws/src/car_ctr/maps/
rosrun map_server map_saver  -f center_map2 map:=/occupancy_map_local`
### 2.2 Run
1. first, move car to the start point of mapping, then log in intranet via Easy Connect
2. run .sh
```sh
cd $PATH_run.sh
bash run.sh
```
3. specify goal point in rviz 




