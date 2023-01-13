#!/usr/bin/env python

#imports
import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import JointState
import paho.mqtt.client as mqtt 
from time import sleep
import math
import random
import json

class NodeClass:
    def __init__(self):
        #initialize node
        rospy.init_node("ur10_joint_angles")
        self.nodename = rospy.get_name()
        rospy.loginfo("Started node %s" % self.nodename)
        
        #vars
        self.percent = 0

        #params and ROS params
        self.rate = rospy.get_param('~rate',100)

        #subscribers
        rospy.Subscriber("brwsrButtons", String, self.callback)
        #publishers
        self.progressPub = rospy.Publisher("progress", Int32, queue_size=10)

    def callback(self, msg):
        if msg.data == 'start_count' and self.percent<100:
            self.percent += 1
        else:
            self.percent = 0

    def spin(self):
        self.r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            percentPubMsg = Int32()
            percentPubMsg.data = self.percent
            self.progressPub.publish(percentPubMsg)
            self.r.sleep()

if __name__ == '__main__':
    """main"""
    try:
        nodeClass = NodeClass()
        nodeClass.spin()
    except rospy.ROSInterruptException:
        pass