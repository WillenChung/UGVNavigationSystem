gnome-terminal --title="rslidar" --tab  -e 'bash -c "roslaunch rslidar_sdk start.launch; exec bash"' \
--tab --title="realsense" -e 'bash -c "sleep 2; roslaunch realsense2_camera rs_camera.launch; exec bash"' \
--tab --title="traversability_mapping" -e 'bash -c "sleep 3; roslaunch traversability_mapping online.launch; exec bash"' \
--tab --title="rosbag_record" -e 'bash -c "sleep 4; cd ~/rosbag/; rosbag record /rslidar_points /tf /camera/color/image_raw -o test_map.bag; exec bash"' \

