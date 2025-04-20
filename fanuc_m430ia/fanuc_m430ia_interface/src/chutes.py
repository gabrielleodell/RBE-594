#!/usr/bin/env python

import os
import sys
import rospy

# TF
import tf2_ros
import tf2_msgs.msg

# Messages
import moveit_msgs.msg
import geometry_msgs.msg

# MoveIt
import moveit_commander

from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest


class FanucInterface(object):

    def __init__(self):

        super(FanucInterface, self).__init__()

        # Sleep time for moving waste
        self.sleep = 0.1

        # Failure count
        self.pick_counter = 0
        self.place_counter = 0

        # Initialize
        rospy.init_node('interface', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Set up config
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('M430ia')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')

        # # Set up end effector
        self.arm.set_end_effector_link('flange')
        self.ee = self.arm.get_end_effector_link()   
        self.ee_links = self.robot.get_link_names('ee_links')

        # Send robot to home
        # self.arm.set_named_target('home')
        # self.gripper.set_named_target('home')
        # self.go_to()

        # Fixed Z heights
        self.z_pick = 1.10      
        self.z_place = 0.75     
        self.z_clear = 1   
        self.z_conveyor = 1.1

        self.arm.set_goal_tolerance(0.05)

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1.25
        self.scene.add_box('conveyor', pose, (0.91, 3, 0.1))

    # Go to target pose or state
    def go_to(self):
        self.arm.go()                               # Wait for move to finish
        self.arm.stop()                             # Stop movement
        self.arm.clear_pose_targets()               # Clean targets

    # Move to desired pose
    def move_to_pose(self, x, y, z):

        pose = Pose()
        pose.position.x = -x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1

        self.arm.set_pose_target(pose)
        return self.go_to()                                                     

    # Place part in waste chute
    def place(self, chute):   

        # Move to safe position above chute
        print('Moving above chute', chute)
        status = self.move_to_pose(chute[0], chute[1], self.z_clear)

        return status

# Main 
if __name__ == '__main__':

    try:

        print('Moving to all chutes')
        test = FanucInterface()

        chutes = [[-0.55, 0.1],
                 [-0.6, 0.35],
                 [0.55, 0.35],
                 [0.6, -0.1],
                 [0.55, -0.4],
                 [-0.56, -0.32]]

        test.place(chutes[0])
        test.place(chutes[1])
        test.place(chutes[2])
        test.place(chutes[3])
        test.place(chutes[4])
        test.place(chutes[5])


        print('Test complete')

    except rospy.ROSInterruptException:
        print('Error occured')

    except KeyboardInterrupt:
        print('Keyboard interrupt')