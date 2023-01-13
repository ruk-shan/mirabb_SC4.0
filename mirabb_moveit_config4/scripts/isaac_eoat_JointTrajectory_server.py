#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult

class FollowJointTrajectoryServer:
    def __init__(self):
        # self.action_server = actionlib.SimpleActionServer("follow_joint_trajectory", FollowJointTrajectoryAction, self.goal_callback, False)
        self.action_server = actionlib.SimpleActionServer("mirabb/eoat_controller/follow_joint_trajectory", FollowJointTrajectoryAction, self.goal_callback, False)
        self.action_server.start()

    def goal_callback(self, goal):
        # Extract the joint trajectory from the goal message
        trajectory = goal.trajectory
        # Iterate through the trajectory points and execute them
        for point in trajectory.points:
            # Move the robot arm to the desired joint positions
            move_arm(point.positions)
            print (f"point:{point}")
        # Set the result of the action to success
        result = FollowJointTrajectoryResult()
        result.error_code = FollowJointTrajectoryResult.SUCCESS
        print (f"result.error_code: {result.error_code}")
        self.action_server.set_succeeded(result)

    def preempt_callback(self):
        # Handle preemption
        self.action_server.set_preempted()
        
    def cancel_callback(self):
        # Handle cancelation
        self.action_server.set_canceled()

def move_arm(joint_positions):
    # Move the robot arm to the desired joint positions
    # Example code to move the robot arm
    pass

if __name__ == "__main__":
    rospy.init_node("eoat_follow_joint_trajectory_server")
    server = FollowJointTrajectoryServer()
    rospy.spin()

