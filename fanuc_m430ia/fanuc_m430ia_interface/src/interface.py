#!/usr/bin/env python

import sys
import rospy

# Messages
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String

# MoveIt
import moveit_commander


class FanucInterface(object):

    def __init__(self):

        super(FanucInterface, self).__init__()

        # Sleep time for moving waste
        self.sleep = 0.1

        # Initialize
        rospy.init_node('interface', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)

        # Set up config
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander('M430ia')

        # Reference
        self.frame = self.arm.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        # Set up end effector
        self.arm.set_end_effector_link('tool0')
        self.ee = self.arm.get_end_effector_link()

        # Premade states
        self.home = [0, -0.7854, 2.3562, -1.5708, 0]
        self.camera = [0, 0, 1.5708, -1.5708, 0]
        self.straight = [0, 0, 0, 0, 0]


    # Go to home position
    def go_home(self):
        self.move_to_state(self.home)


    # Go to camera position
    def go_camera(self):
        self.move_to_state(self.camera)


    # Go to straight position
    def go_straight(self):
        self.move_to_state(self.straight)


    # Move to desired joint state
    def move_to_state(self, state):

        self.arm.set_joint_value_target(state)  # Set target state
        self.go_to()                            # Move to state


    # Move to desired position
    def move_to_pos(self, pos):

        self.arm.set_position_target(pos)       # Set target pose
        self.go_to()                            # Move to pose


    # Go to target
    def go_to(self):

        self.arm.go()                           # Wait for move to finish
        self.arm.stop()                         # Stop movement
        self.arm.clear_pose_targets()           # Clean targets


    # Plan a path
    def plan_path(self):

        return #TODO


    # Plan and execute a path
    def execute_path(self, plan):

        self.arm.execute(plan)                  # Wait for move to finish
        self.arm.stop()                         # Stop movement
        self.arm.clear_pose_targets()           # Clean targets


    # Open the gripper
    def open(self):
        return 


    # Close the gripper
    def close(self):
        return

    # Pick up waste object
    def grab_waste(self, waste):

        self.attach_object(self.ee, waste, touch_links=self.ee)
        rospy.sleep(self.sleep)


    # Drop waste object
    def drop_waste(self, waste):
        self.remove_attached_object(self.ee, waste)
        rospy.sleep(self.sleep)


if __name__ == '__main__':

    try:

        print('Start test')
        test = FanucInterface()

        print('Moving to straight position')
        test.go_straight()

        print('Moving to home position')
        test.go_home()

        print('Move to camera position')
        test.go_camera()

        print('Move to a set state')
        test.move_to_state([1.5708, -0.7854, 2.3562, -0.7854, 1.5708])

        print('Move to a set position')
        test.move_to_pos([0.074, 0.260, 1.100])

        print('Test complete')

    except rospy.ROSInterruptException:
        print('Error occured')

    except KeyboardInterrupt:
        print('Keyboard interrupt')