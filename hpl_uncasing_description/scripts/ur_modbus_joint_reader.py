#!/usr/bin/env python3

#imports
import rospy
from std_msgs.msg import String, Int32, UInt32MultiArray
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pyModbusTCP.client import ModbusClient


class NodeClass:
    def __init__(self):
        #initialize node
        rospy.init_node("joint_pub")
        self.nodename = rospy.get_name()
        robotIP = rospy.get_param("robotIP")
        rospy.loginfo("Started node %s" % self.nodename)
        robotIP  = rospy.get_param("robotIP")
        self.modbus_conn = ModbusClient(host=robotIP, port=502, auto_open=True) #host ip is ROBOT's ip
        self.rate = 50 

        self.urJoinsPub = rospy.Publisher("urJoints", UInt32MultiArray, queue_size=10)


    # def read_UR_joints(self):
    #     while not rospy.is_shutdown():
    #         self.result = self.modbus_conn.read_holding_registers(270,6)
    #         print(self.result)
    
    
    def get_urJoints(self):
        self.result = self.modbus_conn.read_holding_registers(270,6)
        joint_1 = self.result[0] *180/(1000 *math.pi)
        print (f'joint_1:{joint_1}') 
        return self.result

    def spin(self):
        self.r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            urJoints = UInt32MultiArray()
            urJoints.data = self.get_urJoints()
            self.urJoinsPub.publish(urJoints)
            self.r.sleep()




if __name__ == '__main__':
    """main"""
    try:
        nodeClass = NodeClass()
        nodeClass.spin()
    except rospy.ROSInterruptException:
        pass
