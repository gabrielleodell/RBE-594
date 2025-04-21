#!/usr/bin/env python

import os
import sys
import time
import rospy
import pathlib

# TF
import tf2_ros
import tf2_msgs.msg

# Messages
import moveit_msgs.msg
import geometry_msgs.msg

# MoveIt
import moveit_commander

from std_msgs.msg import String
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point, Pose, PoseStamped
from waste_vision.msg import PointArray, ClassifiedPoint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint


class FanucInterface(object):

    def __init__(self):

        super(FanucInterface, self).__init__()

        # Initialize
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

        # Home
        self.move_gripper('home')
        self.go_to_preset('home')

        # Fixed Z heights
        self.z_waste = 1.10    
        self.z_clear = 1.00   
        self.z_chute = 0.95   

        # Tolerances
        self.arm.set_goal_tolerance(0.05)
        self.arm.set_goal_joint_tolerance(0.05)
        self.arm.set_goal_position_tolerance(0.05)
        self.arm.set_goal_orientation_tolerance(0.05)


    # Transform waste location from camera frame to robot frame
    def transform(self, x0, y0):

        # Shift X right
        x = x0 - (0.915 / 2)
        y = y0 - 0.25

        return


    # Find chute coordinates based on chute_id number
    def get_chute(self, chute_id):

        # Wills
        # switch = {
        #     0: [-0.55,  0.10],
        #     1: [-0.60,  0.35],
        #     2: [ 0.55,  0.35],
        #     3: [ 0.55, -0.10],
        #     4: [ 0.55, -0.40],
        #     5: [-0.56, -0.32]
        # }

        # Original
        # switch = {
        #     0: [-0.53,  0.40],
        #     1: [-0.43,  0.00],
        #     2: [-0.53, -0.40],
        #     3: [ 0.53,  0.40],
        #     4: [ 0.53,  0.00],
        #     5: [ 0.53, -0.40]
        # }

        switch = {
            0: [-0.60, -0.40],      # Bottom right
            1: [ 0.60, -0.40],      # Bottom left
            2: [-0.60,  0.40],      # Top right
            3: [-0.60,  0.00],      # Middle right (was 5)
            4: [ 0.60,  0.00],      # Middle left
            5: [ 0.60,  0.40]      # Top left (was 3)
        }

        # Default to bin 0
        return switch.get(chute_id, [-0.55,  0.10])


    # Go to target pose or state
    def go_to(self):
        self.arm.go()                               # Go to target
        self.arm.stop()                             # Stop movement
        self.arm.clear_pose_targets()               # Clean targets


    # Go to preset position (srdf)
    def go_to_preset(self, name):
        self.arm.set_named_target(name)             # Set to srdf group name
        self.go_to()                                # Move to state


    # Move to desired joint state
    def move_to_state(self, state):
        self.arm.set_joint_value_target(state)      # Set target state
        self.go_to()                                # Move to state


    # Move to desired position
    def move_to_position(self, x, y, z):
        self.arm.set_position_target([x, y, z])     # Set target position
        self.go_to()                                # Move to pose


    # Move to desired pose
    def move_to_pose(self, x, y, z):

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1

        self.arm.set_pose_target(pose)
        self.go_to()                                                     


    # Plan and execute a vertical path
    def linear(self, z):

        # Current pose
        start = self.arm.get_current_pose().pose
        z0 = start.position.z

        # New pose
        pose = start

        # Move down
        if z > z0:
            pose.position.z += (z - z0)

        # Move up
        else:
            pose.position.z -= (z0 - z)

        waypoints = []
        waypoints.append(pose)

        self.arm.set_max_velocity_scaling_factor(0.1)

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, avoid_collisions=False)

        # Path found
        if fraction == 1.0:
            self.arm.execute(plan)  
            path = True     

        else:
            print('Failed to find path: ', fraction)
            path = False

        self.arm.stop()                 
        self.arm.clear_pose_targets()    


    # Move the gripper
    def move_gripper(self, name):
        self.gripper.set_named_target(name)
        self.gripper.go() 


    # Pick part up from conveyor
    def pick(self, waste):   

        # Move to safe position above part
        print('Moving to position above')
        self.move_to_pose(waste[0], waste[1], self.z_clear)

        # Open the gripper
        print('Opening gripper')
        self.move_gripper('open')

        # Move down to part
        print('Moving to waste')
        self.move_to_pose(waste[0], waste[1], self.z_waste)

        # Grab part
        print('Grabbing waste')
        self.move_gripper('close')

        # Move up to clear conveyor
        print('Moving up to clear')
        self.move_to_pose(waste[0], waste[1], self.z_clear)


    # Place part in waste chute
    def place(self, chute):   

        # Move to chute
        print('Moving to chute')
        # self.move_to_pose(chute[0], chute[1], self.z_chute - 0.1)
        self.move_to_pose(chute[0], chute[1], self.z_chute)

        # Release part 
        print('Releasing part')
        self.move_gripper('open')

        # Move up to clear
        # self.move_to_pose(chute[0], chute[1], self.z_chute - 0.1)

        # Home robot
        # self.go_to_preset('home')


    # Pick part from conveyor and place in bin
    def sort(self, waste, chute_id):

        # Find chute location
        chute = self.get_chute(chute_id)

        # Start time
        start = time.time()

        print(f'Picking part at {waste}')
        self.pick(waste)

        print(f'Placing part in chute at {chute}')
        self.place(chute)

        # End time
        end = time.time()

        # Cycle time
        cycle = round(end - start, 2)
        print(f'Cycle time: {cycle} sec')


# Callback for vision classifier
def callback(msg, robot):

    # Parse the data
    for point in msg.points:

        print(f'Received request to pick waste from {[point.x, point.y]} and place it in bin [{point.class_id}]')

        # Transform points to robot frame
        x = round(point.x - (0.915 / 2), 4)
        y = round(point.y - 0.25, 4)

        waste = [x, y]
        chute_id = point.class_id

        # Sort the waste
        robot.sort(waste, chute_id)

    print('All waste in photo sorted')


# Main 
if __name__ == '__main__':

    try:

        # Initialize node
        rospy.init_node('interface', anonymous=True)

        # Intiialize robot
        robot = FanucInterface()

        # Initialize vision subscriber
        rospy.Subscriber('trajectory_topic', PointArray, callback, robot)
        rospy.spin()


    except rospy.ROSInterruptException:
        print('Error occurred')

    except KeyboardInterrupt:
        print('Keyboard interrupt')
    
    finally:
        rospy.loginfo('Stopping node')
        rospy.signal_shutdown('Node stopped')
        sys.exit()