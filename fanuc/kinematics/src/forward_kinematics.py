#!/usr/bin/env python

from numpy import pi, sin, cos, tan, sqrt
from moveit_commander import MoveGroupCommander

import rospy
import numpy as np


class GenerateFK:

    # Initialize
    def __init__(self):

        # Robot specs
        self.la = 0.89          # Length of upper joints (J1-3 A)
        self.lb = 1.38          # Length of lower joints (J1-3 B)
        self.le = 0.10          # Side length of the EE triangle
        self.lf = 0.56          # Side length of the fixed triangle

        # Initialize node
        rospy.init_node('FK_node', anonymous=True)
        rospy.spin()


        # Find current state
        move_group = MoveGroupCommander("<<nav!>>Delta<<!/nav>>")
        state = move_group.get_current_joint_values()

        # Current angles
        self.q1 = state[0]     # J1
        self.q2 = state[1]     # J2
        self.q3 = state[2]     # J3


    # Function to calculate the forward kinematics based on q1, q2, and q3 
    def forward(self):

        # Input thetas
        q1 = self.q1       
        q2 = self.q2
        q3 = self.q3

        # Robot specs
        la = self.la
        lb = self.lb
        le = self.le
        lf = self.lf

        # Constants
        t = (lf - le) * sqrt(3)/6 

        # Calculate position of J1
        x1 =  0
        y1 = -(t + (la * cos(q1)))
        z1 = -la * sin(q1)
     
        # Calculate position of J2
        x2 =  (t + (la * cos(q2))) * sin(pi/3)
        y2 =  (t + (la * cos(q2))) * sin(pi/6)
        z2 = -la * sin(q2)
     
        # Calculate position of J3
        x3 = -(t + (la * cos(q3))) * sin(pi/3)
        y3 =  (t + (la * cos(q3))) * sin(pi/6)
        z3 = -la * sin(q3)
     
        # Elbow points 
        w1 = x1**2 + y1**2 + z1**2
        w2 = x2**2 + y2**2 + z2**2
        w3 = x3**2 + y3**2 + z3**2

        # Simplify equation
        i = (y2 - y1)*x3 - (y3 - y1)*x2
         
        # x = a1*z + b1
        a1 =  ((z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1)) / i
        b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / (2 * i)
     
        # y = a2*z + b2
        a2 = -((z2 - z1)*x3 - (z3 - z1)*x2) / i
        b2 =  ((w2 - w1)*x3 - (w3 - w1)*x2) / (2 * i)

        # a*z^2 + b*z + c = 0
        a = a1**2 + a2**2 + 1
        b = 2 * ((a1 * b1) + a2*(b2 - y1) - z1)
        c = b1**2 + (b2 - y1)**2 + z1**2 - lb**2
      
        # Calculate discriminants
        d = b**2 - (4 * a * c)

        # Solution does not exist (singularity)
        if d < 0: 
            return None
     
        # Position
        z = -(b + sqrt(d)) / (2 * a)
        x =  (a1 * z) + b1
        y =  (a2 * z) + b2

        # Return end effector position
        return x, y, z             