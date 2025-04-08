#!/usr/bin/env python

from moveit_commander import MoveGroupCommander

import rospy



# Stops all robot movement
def estop():

    move_group = MoveGroupCommander("<<nav!>>Delta<<!/nav>>")
    move_group.stop()

    rospy.loginfo('Machine is in E-Stop')

    
# Initialize rospy
if __name__ == '__main__':

    # Start node
    rospy.init_node('estop_node', anonymous=True)
    rospy.spin()

    # E-Stop subscriber
    stop = rospy.Subscriber('/estop', String, estop)

    # Print 
    rospy.loginfo('E-Stop topic has started')

