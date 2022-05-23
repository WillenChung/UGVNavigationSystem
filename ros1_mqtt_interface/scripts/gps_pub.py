#!/usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import sys
import signal
import std_msgs.msg
import serial
import time
import threading 
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import struct
import math
import pynmea2
from my_msgs.msg import CarData
from nav_msgs.msg import Odometry

"""
解析GNSS获取的流数据
"""
streamreader = pynmea2.NMEAStreamReader()
"""
解析后的对象包含的属性
(('Timestamp', 'timestamp', <function timestamp at 0x0000017330BA70D8>), ('Status', 'status'), ('Latitude', 'lat'), ('Latitude Direction', 'lat_dir'), 
('Longitude', 'lon'), ('Longitude Direction', 'lon_dir'), ('Speed Over Ground', 'spd_over_grnd', <class 'float'>), ('True Course', 'true_course', <class 'float'>), 
('Datestamp', 'datestamp', <function datestamp at 0x0000017330BC3558>), ('Magnetic Variation', 'mag_variation'), ('Magnetic Variation Direction', 'mag_var_dir'))
"""

def dm2d(input_value):
    #ddmm.mmmmm 度分转度
    ddmm, mmmm = math.modf(input_value)
    dd , mm = math.modf(ddmm / 100.0)
    output_value_degree = dd + (mm * 100 + mmmm) / 60.0
    
    return output_value_degree

# 阻塞监听话题线程
def thread_job():
    rospy.spin()

# 退出程序执行
def sigint_handler(signal, frame):   
    print(" ")
    print("Interrupt!\n")
    print("Terminated")
    ser.flush()
    time.sleep(1)
    if ser.isOpen():
        ser.close()
    sys.exit(0)

#serial init
node_name='GNSS_node' 
serialPort = "/dev/gps_uart"
baudRate = 38400
ser = serial.Serial(serialPort, baudRate)
serial.bytesize = 8
serial.stopbits = 1
print("GNSS serial port is %s ,baudRate is %d" % (serialPort, baudRate))
signal.signal(signal.SIGINT, sigint_handler)
local_x = 0.0
local_y = 0.0

def odomCallback(msg):
    global local_x
    global local_y
    local_x = msg.pose.pose.position.z
    local_y = msg.pose.pose.position.x
    
def communication():
    cardata = CarData()
    global local_x
    global local_y
    # cardata.seq = 1
    # cardata.timestamp = "0"
    # cardata.longitude = "0"
    # cardata.latitude = "0"
    rospy.init_node(node_name, anonymous=True)  
    # print("init_node:",node_name)
    # 发布话题
    gnss_data_pub = rospy.Publisher("/gnss_data", CarData, queue_size= 10)   # 经纬度发布
    rospy.Subscriber("/integrated_to_init", Odometry, odomCallback)
    # 订阅话题spin线程
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()

    rate = rospy.Rate(1)
    global ser
    count = 0
    seq = 0
    while not rospy.is_shutdown():
        try:
            if count!=0:
                ser = serial.Serial(serialPort, baudRate)
                print("restart GNSS serial")
                ser.timeout = 2
            count+=1
            while not rospy.is_shutdown():
                # print(local_x)
                # print(local_y)
                data_generator = ser.read(ser.in_waiting)
                #print(data_generator)
                for data in data_generator:
                    print(data)
                    # print(type(data))
                    for msg in streamreader.next(data.decode()):
                        print(msg)
                        if (msg.identifier() == 'GNRMC,'):
                            #print(msg.lon)
                            #print({msg.lat})
                            #     print( {msg.datestamp}-{msg.timestamp})
                            #     print({msg.spd_over_grnd})
                            #     print({msg.true_course})
                            #     print({msg.mag_variation})
                            #     print({msg.mag_var_dir}\
                            # print(msg.status)
                            # print(type(msg.status))
                            # print(type(msg.timestamp))
                            # print(type(msg.longitude))
                            # print(type(msg.latitude))
                            # print(type(msg.spd_over_grnd))
                            # <type 'unicode'>
                            # type 'datetime.time'>
                            # type 'float'>
                            # <type 'float'>
                            # <type 'float'>
                            #print(type(dm2d(msg.lon)))
                            seq += 1
                            if(msg.status == 'V'):
                                # print("推荐定位信息无效", msg)
                                cardata.seq = seq      
                                cardata.timestamp = ""
                                cardata.longitude = 0
                                cardata.latitude = 0
                                cardata.spd_over_grnd = 0
                                cardata.local_x = local_x
                                cardata.local_y = local_y
                            else:
                                cardata.seq = seq
                                cardata.timestamp = str(msg.timestamp)
                                cardata.longitude = msg.longitude
                                cardata.latitude = msg.latitude
                                cardata.spd_over_grnd = float(msg.spd_over_grnd * 1.85 / 3.6)
                                cardata.local_x = local_x
                                cardata.local_y = local_y
                                
                                filename = '/home/willen/test_ws/src/ros1_mqtt_interface/scripts/gps_value.txt'  
                                with open(filename, 'a') as file_object:
                                    file_object.write(str(msg.status))
                                    file_object.write(' ')
                                    file_object.write(str(msg.datestamp))
                                    file_object.write(' ')
                                    file_object.write(str(msg.timestamp))
                                    file_object.write(' ')
                                    file_object.write(str(msg.longitude))
                                    file_object.write(' ')
                                    file_object.write(str(msg.latitude))
                                    file_object.write(' ')
                                    file_object.write(str(msg.spd_over_grnd * 1.85 / 3.6))
                                    file_object.write(' ')
                                    file_object.write(str(local_x))
                                    file_object.write(' ')
                                    file_object.write(str(local_y))
                                    file_object.write(' ')
                                    file_object.write(str(seq))
                                    file_object.write('\n')           
         
                gnss_data_pub.publish(cardata)                                          
                rate.sleep()
                
        except Exception as e:
            print("gnss have a serial error:",e)
            if ser.isOpen():
                ser.close()       


if __name__ == '__main__':
    communication()
    rospy.spin()

 
########################