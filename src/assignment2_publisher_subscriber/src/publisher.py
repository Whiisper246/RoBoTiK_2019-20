#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SpeedCommand
from autominy_msgs.msg import NormalizedSteeringCommand

rospy.init_node('publisher', anonymous=True)

pub = rospy.Publisher('/actuators/steering_normalized',NormalizedSteeringCommand, queue_size=10)
pub2 = rospy.Publisher('/actuators/speed',SpeedCommand, queue_size=10)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
        str= NormalizedSteeringCommand()
        str.value = 1.0
        speed= SpeedCommand()
        speed.value= 0.3 
        rospy.loginfo(str)
        rospy.loginfo(speed)
        pub.publish(str)
        pub2.publish(speed)

