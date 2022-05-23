gnome-terminal --tab -e 'bash -c "roslaunch rslidar_sdk start.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; ffmpeg -f video4linux2 -i /dev/video0 -f flv rtmp://113.54.154.210:1004/hls/video1; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch lego_loam run.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch car_ctr move_base.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch car_ctr car_ctr.launch; exec bash"' \
--tab -e 'bash -c "sleep 1; roslaunch ros1_mqtt_interface communication.launch; exec bash"' \