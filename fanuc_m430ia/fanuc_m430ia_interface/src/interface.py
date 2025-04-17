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
        self.gripper = moveit_commander.MoveGroupCommander('Gripper')

        # Set up end effector
        self.arm.set_end_effector_link('ee_base')
        self.ee = self.arm.get_end_effector_link()   
        self.ee = self.robot.get_link_names('Gripper')


    # Go to home position
    def go_home(self):
        self.arm.set_named_target('home')
        self.go_to()


    # Go to camera position
    def go_camera(self):
        self.arm.set_named_target('camera')
        self.go_to()


    # Go to straight position
    def go_straight(self):
        self.arm.set_named_target('straight')
        self.go_to()


    # Go to target
    def go_to(self):
        self.arm.go()                               # Wait for move to finish
        self.arm.stop()                             # Stop movement
        self.arm.clear_pose_targets()               # Clean targets


    # Move to desired joint state
    def move_to_state(self, state):
        self.arm.set_joint_value_target(state)      # Set target state
        self.go_to()                                # Move to state


    # Move to desired position
    def move_to_pos(self, pos):
        self.arm.set_position_target(pos)           # Set target pose
        self.go_to()                                # Move to pose


    # Plan and execute a path
    def execute_path(self, plan):
        self.arm.execute(plan)                      # Wait for move to finish
        self.arm.stop()                             # Stop movement
        self.arm.clear_pose_targets()               # Clean targets


    # Open the gripper
    def open_gripper(self):
        self.gripper.set_named_target('open')
        self.gripper.go()                          


    # Close the gripper
    def close_gripper(self):
        self.gripper.set_named_target('close')
        self.gripper.go()  


    # Pick part up conveyor
    def pick(self, xy, waste):

        clear = [xy[0], xy[1], xy[2] - 0.25]

        # Move to safe position above part
        self.move_to_pos(clear)

        # Open the gripper
        self.open_gripper()

        # Move down to part
        self.move_to_pos(xy)

        # Pick up part
        #self.attach_object(self.ee, waste, touch_links=self.ee)
        rospy.sleep(self.sleep)
        self.close_gripper()

        # Move up to clear conveyor
        self.move_to_pos(clear)


    # Place part in bin/chute
    def place(self, xy, chute):

        clear = [xy[0], xy[1], xy[2] - 0.25]
        self.remove_attached_object(self.ee, waste, touch_links=self.ee)



if __name__ == '__main__':

    try:

        print('Start test')
        test = FanucInterface()

        print('Moving to home position')
        test.go_home()

        print('Move to camera position')
        test.go_camera()

        print('Open gripper')
        test.open_gripper()

        print('Move to a set state')
        test.move_to_state([1.5708, -0.7854, 2.3562, -0.7854, 1.5708])

        print('Close gripper')
        test.close_gripper()

        print('Move to a set position')
        test.pick([0.074, 0.26, 0.8], 'chute')

        #test.move_to_pos([0.074, 0.260, 1.100])

        print('Open gripper')
        test.open_gripper()

        print('Test complete')

    except rospy.ROSInterruptException:
        print('Error occured')

    except KeyboardInterrupt:
        print('Keyboard interrupt')