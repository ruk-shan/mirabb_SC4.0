#!/usr/bin/env python3



import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("test_rosbridge", anonymous=True)

pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
joint_state = JointState()


# joint_state.name = [
#     "panda_joint1",
#     "panda_joint2",
#     "panda_joint3",
#     "panda_joint4",
#     "panda_joint5",
#     "panda_joint6",
#     "panda_joint7",
#     "panda_finger_joint1",
#     "panda_finger_joint2",
# ]

joint_state.name = ["joint3",
                    "joint4",
                    "joint2",
                    "joint5",
                    "joint1",
                    "joint6",
                    "joint_Base_LeftWheel",
                    "joint_Base_RightWheel",
                    "joint_Base_front_left_caster",
                    "joint_Base_front_right_caster",
                    "joint_Base_rear_left_caster",
                    "joint_Base_rear_right_caster",
                    "joint10",
                    "joint8",
                    "joint9"]


num_joints = len(joint_state.name)


# make sure kit's editor is playing for receiving messages ##

joint_state.position = np.array([0.0] * num_joints)
default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement
max_joints = np.array(default_joints) + 0.5
min_joints = np.array(default_joints) - 0.5

# position control the robot to wiggle around each joint
time_start = time.time()
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints
    pub.publish(joint_state)
    rate.sleep()
