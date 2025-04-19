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
        self.z_pick = 1.30      
        self.z_place = 1.25     
        self.z_clear = 1.20     
        self.z_conveyor = 1.30

        # Demo waste
        self.glass = [-0.10, 0.20, 'glass']
        self.plastic = [-0.10, 0.0, 'plastic']
        self.alumninum = [0.10, 0.20, 'alumninum']
        self.contaminate = [0.0, -0.28, 'contaminate']



    # Spawns waste on conveyor 
    def spawn_waste(self, waste):

        # Set position and orientation
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = waste[0]
        pose.pose.position.y = waste[1]
        pose.pose.position.z = 1.40
        pose.pose.orientation.w = 1.0

        # Get mesh file
        folder = os.path.dirname(os.path.abspath(__file__))
        folder = os.path.dirname(folder)
        folder = os.path.dirname(folder)
        path = os.path.join(folder, 'fanuc_simulation', 'Mesh', waste[2])

        # Add it to scene
        self.scene.add_mesh(waste[2], pose, path, size=(0.1, 0.1, 0.1))


    # Go to target pose or state
    def go_to(self):
        self.arm.go()                               # Wait for move to finish
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
        self.arm.set_position_target([x, y, z])
        self.go_to()       


    # Move to desired pose
    def move_to_pose(self, x, y, z):

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1

        self.arm.set_pose_target(pose)
        return self.go_to()                                                     


    # Plan and execute a path
    def linear(self, z):

        # Current pose
        start = self.arm.get_current_pose().pose
        z0 = start.position.z

        # New pose
        pose = start

        if z > z0:
            pose.position.z += (z - z0)

        else:
            pose.position.z -= (z0 - z)

        # Add waypoints
        waypoints = []
        waypoints.append(start)
        waypoints.append(pose)

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, 0.01, avoid_collisions=True)

        # Path found
        if fraction == 1.0:
            self.arm.execute(plan)  
            path = True     

        else:
            print('Failed to find path')
            path = False

        self.arm.stop()                 
        self.arm.clear_pose_targets()    

        return path 


    # Open the gripper
    def open_gripper(self):
        self.gripper.set_named_target('open')
        self.gripper.go()                          


    # Close the gripper
    def close_gripper(self):
        self.gripper.set_named_target('close')
        self.gripper.go()  


    # Pick part up from conveyor
    def pick(self, waste):   

        # Move to safe position above part
        print('Moving above waste')
        status1 = self.move_to_pose(waste[0], waste[1], self.z_clear)

        # Open the gripper
        print('Opening gripper')
        self.open_gripper()

        # Move down to part
        print('Moving down to waste')
        status2 = self.linear(self.z_pick)

        # Grab part
        print('Grabbing waste')
        self.close_gripper()

        # Move up to clear conveyor
        print('Moving up to clear')
        status3 = self.linear(self.z_clear)

        # Check if it succeeded 
        return status1 and status2 and status3


    # Place part in waste chute
    def place(self, waste, chute):   

        # Move to safe position above chute
        print('Moving above chute')
        status1 = self.move_to_pose(chute[0], chute[1], self.z_clear)

        # Move down to chute
        print('Moving down to chute')
        status2 = self.linear(self.z_place)

        # Release part 
        print('Releasing part')
        self.open_gripper()

        # Move up to clear position
        print('Moving up to clear')
        status3 = self.linear(self.z_clear)

        # Home robot
        # self.go_to_preset('home')

        # Check if it succeeded
        return status1 and status2 and status3


    # Pick part from conveyor and place in bin
    def sort(self, waste, chute):

        # Try to pick
        if self.pick(waste) == False:
            self.pick_counter += 1

            print('Pick failed')
            return False

        print('Pick complete')

        # Try to place
        if self.place(waste, chute) == False:
            self.place_counter += 1

            print('Place failed')
            return False

        print('Place complete')

        # Sorting successful
        return True


# Main 
if __name__ == '__main__':

    try:

        print('Start test')
        test = FanucInterface()

        # waste = [-0.337, -0.094, 'glass']
        # chute = [-0.101, -0.068]

        # waste = [-0.101, -0.068, 'glass']
        # chute = [-0.55, 0.0]

        waste = [-0.101, -0.068, 'glass']
        chute = [-0.5, 0.5]

        test.sort(waste, chute)


        print('Test complete')

    except rospy.ROSInterruptException:
        print('Error occured')

    except KeyboardInterrupt:
        print('Keyboard interrupt')