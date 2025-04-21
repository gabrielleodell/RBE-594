#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID


# Main
if __name__ == '__main__':

    try:
        # Initialize
        rospy.init_node('estop', anonymous=True)
        move = rospy.Publisher('/move_group/cancel', GoalID, queue_size=10)
        stop = rospy.Publisher('/estop', Bool, queue_size=10)

        input('\nHit ENTER to activate E-Stop button')

        # Publish empty goal
        move.publish(GoalID())

        # Stop robot and conveyor
        stop.publish(True)

        print('\nEMERGENCY STOP TRIGGERED')


    except rospy.ROSInterruptException:
        rospy.logerr('Error occurred')