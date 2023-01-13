#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal

class ABBDriver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("abb_driver")
        self.joint_states = JointState()
        self.joint_states.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        self.goal_sub = rospy.Subscriber("/mirabb/abb_arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, self.goal_callback)
        self.rate = rospy.Rate(10)
    def goal_callback(self, goal):
        # Extract the joint trajectory from the goal message
        trajectory = goal.goal.trajectory
        # Iterate through the trajectory points and move the robot arm
        for i in range(len(trajectory.points)):
            point = trajectory.points[i]
            self.joint_states.position = point.positions
            self.joint_pub.publish(self.joint_states)
            self.rate.sleep()
    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

if __name__ == "__main__":
    driver = ABBDriver()
    driver.run()