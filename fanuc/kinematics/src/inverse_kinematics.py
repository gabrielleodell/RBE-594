#!/usr/bin/env python

from moveit_commander import MoveGroupCommander
from numpy import cos, sin, sqrt, arctan2, isnan

import rospy
import numpy as np


class GenerateIK:
    def __init__(self):

        # Robot specs
        self.la = 0.89          # Length of upper joints (J1-3 A)
        self.lb = 1.38          # Length of lower joints (J1-3 B)
        self.le = 0.10          # Side length of the EE triangle
        self.lf = 0.56          # Side length of the fixed triangle

        # Find current EE pose
        pose = current_state()

        # Current angles
        self.x = pose[0]       # J1
        self.y = pose[1]       # J2
        self.z = pose[2]       # J3


    # Function to get current joint state of q1, q2, and q3
    def current_pose():

        # Initialize node
        rospy.init_node('IK_node', anonymous=True)

        # Find current state
        move_group = MoveGroupCommander("<<nav!>>EE<<!/nav>>")
        pose = move_group.get_current_pose()

        return pose


    # Function to calculate joint angle based on pose
    def find_angle(self, x0, y0, z0):

        # Robot specs
        la = self.la
        lb = self.lb
        le = self.le
        lf = self.lf

        # Subtract EE length from initial y position
        y0 -= le * (sqrt(3) / 6)

        # Set next y position as base length
        y1 = -lf * (sqrt(3) / 6)

        # z = by + a
        a = (x0**2 + y0**2 + z0**2 + la**2 - lb**2 - y1**2) / (2 * z0)
        b = (y1 - y0) / z0

        # Calculate discriminant
        d = -(a + b*y1)**2 + (b**2 * la**2) + la**2

        # Solution does not exist (singularity)
        if d < 0: 
            return nan

        # Calculate for YZ plane
        y = (y1 - (a * b) - sqrt(d)) / (b**2 + 1)
        z = (b * y) + a

        # Calculate theta
        q = arctan2(-z, y1 - y)

        # Offset by 180 degrees
        if y > y1:
            q += pi

        return q


    # Function to calculate the inverse kinematics based on pose
    def inverse(self):

        # Input pose
        x = self.x
        y = self.y
        z = self.z

        # Offset angle for joints on fixed base
        theta = (2 * pi) / 3

        # Calculate joint angles
        try:

            q1 = find_angle(x, y, z)
            q2 = find_angle((x * cos(theta)) + (y * sin(theta)), (y * cos(theta)) - (x * sin(theta)), z)    # Offset +120 deg
            q3 = find_angle((x * cos(theta)) - (y * sin(theta)), (y * cos(theta)) + (x * sin(theta)), z)    # Offset -120 deg

        except rospy.ROSInterruptException:
            return nan, nan, nan

        return q1, q2, q3


    # Check if the angle is valid
    def valid_angle(self, q):

        return not np.isnan(q)


    # Check if the solution is valid
    def valid_angle(self, q1, q2, q3):

        return not np.isnan(q1), not np.isnan(q2), not np.isnan(q3)
