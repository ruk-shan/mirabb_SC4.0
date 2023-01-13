#!/usr/bin/env python3

#imports
import rospy
from std_msgs.msg import String, Int32
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import paho.mqtt.client as mqtt_client 
from time import sleep
import json


class NodeClass:
    def __init__(self):
        #initialize node
        rospy.init_node("joint_updater")
        self.nodename = rospy.get_name()
        rospy.loginfo("Started node %s" % self.nodename)
        
        #vars
        self.rate = 200
        self.counter = 0
        self.msg_received = {}
    
        #publishers
        self.JointPub = rospy.Publisher('joint_states', JointState, queue_size=10)


    #MQTT call back on message receive ----------------------------------------------------------------
    def on_message(self,client, userdata, msg):  # The callback for when a PUBLISH message is received from the server.
        self.msg_received = json.loads((msg.payload).decode('utf-8'))

    def on_connect(self,client, userdata, flags, rc):  # The callback for when the client connects to the broker
        print("Connected with result code {0}".format(str(rc)))  # Print result of connection attempt
        client.subscribe("uncasing_station/UR10/joint_angles")  # Subscribe to the topic “digitest/test1”, receive any messages published on it

    def socket_client_mqtt_sub_conn(self):
        self.client = mqtt_client.Client("digi_mqtt_test")  # Create instance of client with client ID “digi_mqtt_test”
        self.client.on_connect = self.on_connect  # Define callback function for successful connection
        self.client.on_message = self.on_message  # Define callback function for receipt of a message
        # client.connect("m2m.eclipse.org", 1883, 60)  # Connect to (broker, port, keepalive-time)
        self.client.connect("192.168.68.116",11883)
        self.client.loop_start()  # Start networking daemon
    #---------------------------------------------------------------------- 

    def model_rotator(self):
        self.counter = self.counter + .2/self.rate
        # print (self.counter)
        return self.counter  



    def spin(self):
        self.r = rospy.Rate(self.rate)
        self.socket_client_mqtt_sub_conn()#MQTT

        while not rospy.is_shutdown():
            JointPub = JointState()
            JointPub.header = Header()
            JointPub.header.stamp = rospy.Time.now()
            JointPub.name = ['urJ_01', 'urJ_02', 'urJ_03', 'urJ_04', 'urJ_05', 'urJ_06', 'base_rev']
            # JointPub.position = [0,1,0,0,0,0,self.model_rotator()/math.pi]
            joints = self.msg_received
            # print (f"joints: {joints}")
            try:
                JointPub.position = [joints['joint_01'],
                                    joints['joint_02'],
                                    joints['joint_03'],
                                    joints['joint_04'],
                                    joints['joint_05'],
                                    joints['joint_06'],
                                    self.model_rotator()/math.pi]
                # print (JointPub.position)
            except:
                JointPub.position = [0,1,0,0,0,0,self.model_rotator()/math.pi]
                print ("Missing data")
            JointPub.velocity = []
            JointPub.effort = []
            self.JointPub.publish(JointPub)
            self.r.sleep()

            try:
                # print (self.msg_received["joint_01"])
                pass
            except:
                # print ("error")
                pass
            # pub.publish(JointPub)
            # sleep(1)

if __name__ == '__main__':
    """main"""
    try:
        nodeClass = NodeClass()
        nodeClass.spin()
    except rospy.ROSInterruptException:
        pass








