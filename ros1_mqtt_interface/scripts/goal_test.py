#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
temp_x = 1000.0
temp_y = 1000.0
def talker():
    global temp_x
    global temp_y
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = 4.0
    goal.pose.position.y = 0.0
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    # while not rospy.is_shutdown():
    #     if(temp_x != goal.pose.position.x or temp_y != goal.pose.position.y):
    #         temp_x = goal.pose.position.x
    #         temp_y = goal.pose.position.y
    #         pub.publish(goal)
    #         rospy.loginfo(goal)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass