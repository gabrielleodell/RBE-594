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

from std_msgs.msg import Bool, String
from actionlib_msgs.msg import GoalID
from conveyorbelt_msgs.msg import ConveyorBeltState
from conveyorbelt_msgs.srv import ConveyorBeltControl
from geometry_msgs.msg import Point, Pose, PoseStamped
from waste_vision.msg import PointArray, ClassifiedPoint


class FanucInterface(object):

    def __init__(self):

        super(FanucInterface, self).__init__()

        # Initialize cycle signal
        self.pub = rospy.Publisher('/cycle_start', Bool, queue_size=10)

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

        # For logging
        print('')

        # Send ready signal
        rospy.loginfo('Waiting for next batch...')
        self.pub.publish(True)


    # Find chute coordinates based on chute_id number
    def get_chute(self, chute_id):

        # Original
        switch = {
            0: [-0.60, -0.40],      # Bottom right
            1: [ 0.60, -0.40],      # Bottom left
            2: [-0.60,  0.40],      # Top right
            3: [-0.60,  0.00],      # Middle right
            4: [ 0.60,  0.00],      # Middle left
            5: [ 0.60,  0.40]       # Top left (SPARE)
        }

        # Default to bin 5
        return switch.get(chute_id, [0.60,  0.40])


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


    # Plan and execute a linear path
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
            rospy.loginfo('Failed to find path: ', fraction)
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
        self.move_to_pose(waste[0], waste[1], self.z_clear)

        # Open the gripper
        self.move_gripper('open')

        # Move down to part
        self.move_to_pose(waste[0], waste[1], self.z_waste)

        # Grab part
        self.move_gripper('close')

        # Move up to clear conveyor
        self.move_to_pose(waste[0], waste[1], self.z_clear)


    # Place part in waste chute
    def place(self, chute):   

        # Move to chute
        self.move_to_pose(chute[0], chute[1], self.z_chute)

        # Release part 
        self.move_gripper('open')


    # Pick part from conveyor and place in bin
    def sort(self, waste, chute_id):

        # Find chute location
        chute = self.get_chute(chute_id)

        # Start time
        start = time.time()

        rospy.loginfo(f'Picking part at {waste}')
        self.pick(waste)

        rospy.loginfo(f'Placing part in chute at {chute}')
        self.place(chute)

        # End time
        end = time.time()

        # Cycle time
        cycle = round(end - start, 2)
        rospy.loginfo(f'Cycle time: {cycle} sec')


# Callback for vision classifier
def vision_callback(msg, robot):

    rospy.loginfo('New batch received')

    # Stop vision until batch is sorted
    robot.pub.publish(False)

    # Parse the data
    for point in msg.points:

        rospy.loginfo(f'Received request to pick waste from {[point.x, point.y]} and place it in bin [{point.class_id}]')

        # Transform points to robot frame
        x = round(point.x - (0.915 / 2), 4)
        y = round(point.y - 0.25, 4)

        waste = [x, y]
        chute_id = point.class_id

        # Sort the waste
        robot.sort(waste, chute_id)

    rospy.loginfo('Batch complete')
    print('')

    # Home the robot
    robot.go_to_preset('home')

    # Ready for next batch
    rospy.loginfo('Waiting for next batch...')
    robot.pub.publish(True)


# Callback for E-Stop button
def estop_callback(msg, robot):

    rospy.loginfo('E-STOP TRIGGERED')

    # Stop robot movement
    robot.arm.stop()
    robot.gripper.stop()

    # Stop conveyor movement
    conveyor = rospy.ServiceProxy('CONVEYORPOWER', ConveyorBeltControl)
    conveyor(0.0)

    # Stop node
    rospy.signal_shutdown('E-STOP TRIGGERED')


# Main 
if __name__ == '__main__':

    try:

        # Initialize node
        rospy.init_node('interface', anonymous=True)

        # Intiialize robot
        robot = FanucInterface()

        # Initialize e-stop subscriber
        rospy.Subscriber('estop', Bool, estop_callback, robot)

        # Initialize vision subscriber
        rospy.Subscriber('trajectory_topic', PointArray, vision_callback, robot)

        # Refresh
        rospy.spin()


    except Exception as e:
        rospy.logerr(f'Exception occurred: {e}')

    except KeyboardInterrupt:
        rospy.logerr('Keyboard interrupt')
    
    finally:
        rospy.signal_shutdown('Node stopped')
        sys.exit()