#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####               mqtt接收界面信息，发布给Ros其他节点                ###
########################################################################
# 该例程将使用mqtt订阅界面的/ros_interface_data话题，并发布给其他ros节点
from re import T
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
from geometry_msgs.msg import PoseStamped
temp_x = 1000.0
temp_y = 1000.0
reload(sys)
sys.setdefaultencoding('utf8')

######################################################################
######################################################################
###################### ROS_Publisher初始化 ############################
# pub = rospy.Publisher('/interface_data', String, queue_size=10)
pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
rospy.init_node('mqtt_talker1', anonymous=True)
goal = PoseStamped()
goal.header.seq = 0
goal.header.stamp = rospy.Time.now()
goal.header.frame_id = "map"
# goal.pose.position.x = 0.0
# goal.pose.position.y = 0.0
goal.pose.position.z = 0.0
goal.pose.orientation.x = 0.0
goal.pose.orientation.y = 0.0
goal.pose.orientation.z = 0.0
goal.pose.orientation.w = 1.0


######################################################################
######################################################################
###################### MQTT连接服务器端口和IP #########################
MQTTHOST = '113.54.154.210'  #icv外部IP
#MQTTHOST = '192.168.1.123'  #icv内部IP
#MQTTHOST = 'localhost'  #本地IP
MQTTPORT = 1003   #icv外部端口
#MQTTPORT = 1883   #icv内部端口

######################### MQTT声明用户端名称 ###########################
mqttClient = mqtt.Client(client_id="InterfaceToRos_001")

######################### 连接MQTT服务器 ##############################
def on_mqtt_connect():
	mqttClient.connect(MQTTHOST, MQTTPORT, 60)
	mqttClient.loop_start()

	
######################### MQTT_消息回调函数 ##############################
def on_message_come(client, userdata, msg):
	global temp_x
	global temp_y
	global goal
	msgs = json.loads(str(msg.payload.decode())) #str转dict
	print(msgs)
	# print(msgs["interface_target_GPS_"])
	# print(msgs["interface_target_GPS_"].split('\n', 1)[0])
	# print(msgs["interface_target_GPS_"].split('\n', 1)[1])
	if(msgs["interface_target_GPS_"] != ''): 
		goal.pose.position.x = float(msgs["interface_target_GPS_"].split(' ', 1)[0])
		goal.pose.position.y = float(msgs["interface_target_GPS_"].split(' ', 1)[1])
	########## 每次mqtt收到消息，ros发布消息 #############
	# if(temp_x != goal.pose.position.x or temp_y != goal.pose.position.y):
	# 	temp_x = goal.pose.position.x
	# 	temp_y = goal.pose.position.y
	# 	pub.publish(goal)
	

    
    
######################### MQTT_sub消息 ##############################
def on_subscribe():
    mqttClient.subscribe("/interface_ros_data", 0)#chat为话题名称
	# mqttClient.subscribe("/", 0)
    mqttClient.on_message = on_message_come # 消息到来处理函数


def client_node(): 
	on_mqtt_connect()
	on_subscribe()
	rate = rospy.Rate(1) 
	while not rospy.is_shutdown():
		rospy.loginfo(goal)
		pub.publish(goal)
		rate.sleep()
  
if __name__ == '__main__':
	#print(time.time())
	client_node()
	print("connect successful!")

    
    
    
    
    
    
    
    
    
    
    
    
    
