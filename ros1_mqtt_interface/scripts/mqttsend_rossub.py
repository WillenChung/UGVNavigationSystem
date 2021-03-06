#!/usr/bin/env python
# -*- coding: utf-8 -*-






########################################################################
####                接受ROS内部话题，使用mqtt发送给界面                ###
########################################################################
# 该例程将使用mqtt发布/ros_interface_data话题，将接收的ROS信息使用json格式发送给界面
from sqlite3 import Timestamp
import rospy
import message_filters
import socket
import time
import sys
from std_msgs.msg import String
######## MQTT的有关引用 #############
import paho.mqtt.client as mqtt
import json
import random
from my_msgs.msg import CarData



######################################################################
######################################################################
###################### MQTT连接服务器端口和IP #########################
MQTTHOST = '113.54.154.210'  #icv外部IP
# MQTTHOST = '192.168.50.93'  #icv内部IP
#MQTTHOST = 'localhost'  #本地IP
MQTTPORT = 1003   #icv外部端口
# MQTTPORT = 1883   #icv内部端口

######################### MQTT声明用户端名称 ##############################
mqttClient = mqtt.Client(client_id="RosToInterface_001")

######################### 连接MQTT服务器 ##############################
def on_mqtt_connect():
	mqttClient.connect(MQTTHOST, MQTTPORT, 60)
	mqttClient.loop_start()
	
######################### MQTT_pub消息 ##############################
def on_publish(topic, payload, qos):
	mqttClient.publish(topic, payload, qos)
	




######################################################################
######################################################################
######################### MQTT_pub消息变量声明 ########################
car_speed = ""
plane_speed = ""

############################# ROSsub回调函数 ###################
def ros1_dataCallback(data1):
	# global timestamp
	# global longitude
	# global latitude
	seq = data1.seq
	timestamp = data1.timestamp
	longitude = data1.longitude
	latitude = data1.latitude
	car_spd = data1.spd_over_grnd
	local_x = data1.local_x
	local_y = data1.local_y
	########## ros收到消息，mqtt发布消息 #############
	# object = {'timestamp' : timestamp, 'longitude' : longitude, 'latitude': latitude,'car_spd': car_spd, 'seq': seq}
	object = {'timestamp' : timestamp, 'gps_value' : str(longitude)+','+str(latitude), 'local_xy': str(local_x)+','+str(local_y), 'car_spd': str(car_spd), 'seq': str(seq)}
	on_publish('/ros_interface_data',json.dumps(object),0)
	print('mqtt send /ros_interface_data: '+str(object))
	# rospy.loginfo("Subcribe ros_interface_data:   car_speed:  %s       plane_speed:  %s" , car_speed, plane_speed)


def ros2_dataCallback(data1):
	global car_speed
	global plane_speed
	plane_speed = data1.data
	########## ros收到消息，mqtt发布消息 #############
	object = {"car_speed" : car_speed,"plane_speed" : plane_speed,}
	on_publish('/ros_interface_data',json.dumps(object),0)
	print('matt send /ros_interface_data: '+str(object))
	rospy.loginfo("Subcribe ros_interface_data:   car_speed:  %s       plane_speed:  %s" , car_speed, plane_speed)

	
	
########################## ROSsub ##############################
def ros1_subscriber():
	#ROS节点初始化
	rospy.init_node('ros1_subscriber', anonymous=True)
	##同时订阅/car_speed和/plane_speed 两个话题
	rospy.Subscriber("/gnss_data", CarData, ros1_dataCallback)
	rospy.Subscriber("/plane_speed", String, ros2_dataCallback)
	rospy.spin()




######################################################################
######################################################################
	



if __name__ == '__main__':
	on_mqtt_connect()
	ros1_subscriber()










