#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import numpy as np
from ppr.robot import Robot

class Robot_3R(Robot):
    def __init__(self, link_length, link_width):
        """ Robot with a rectangle for every link """
        
        # all input lists must have the same length
        self.ndof = 3
        self.jt = ['r', 'r', 'r']
        self.d = link_length
        self.a = [0, 0, 0]
        
        # default collision shapes
        self.lx = [0] * self.ndof
        self.ly = [0] * self.ndof
        self.lw = link_width
        self.ll = link_length
        self.adapt_ll = True
    
    def ik(self, p):
        """ Analytic inverse kinematics for 3 link robot """
        
        # define variables for readability
        l1, l2, l3 = (self.ll[0], self.ll[1], self.ll[2])
        x, y, phi = (p[0], p[1], p[2])
        
        # initialize output
        q_up = [0, 0, 0]
        q_do = [0, 0, 0]
        reachable = False
        
        # start calculations
        if (l1 + l2 + l3) >= np.sqrt(x**2 + y**2):
            # coordinates of end point second link
            pwx = x - l3 * np.cos(phi)
            pwy = y - l3 * np.sin(phi)
            R2 = pwx**2 + pwy**2
            if (l1 + l2) >= np.sqrt(R2):
                reachable = True
                # calculate q2
                c2 = (R2 - l1**2 - l2**2) / (2*l1*l2)
                s2 = np.sqrt(1 - c2**2)
                q_up[1] = np.arctan2(s2, c2) # elbow up
                q_do[1] = np.arctan2(-s2, c2) # elbow down
                # calculate q1
                temp = (l1 + l2 * c2)
                s1_up = (temp * pwy - l2 * s2 * pwx) / R2
                c1_up = (temp * pwx + l2 * s2 * pwy) / R2
                s1_do = (temp * pwy + l2 * s2 * pwx) / R2
                c1_do = (temp * pwx - l2 * s2 * pwy) / R2
                q_up[0] = np.arctan2(s1_up, c1_up)
                q_do[0] = np.arctan2(s1_do, c1_do)
                # finally q3
                q_up[2] = phi - q_up[0] - q_up[1]
                q_do[2] = phi - q_do[0] - q_do[1]

        if reachable:
            return {'success': True, 'q': [q_up, q_do]}
        else:
            return {'success': False, 'info': "unreachable"}

class Robot_2P3R(Robot):
    def __init__(self, link_length):
        """ Robot with a rectangle for every link """
        
        # all input lists must have the same length
        self.ndof = 5
        self.jt = ['p', 'p', 'r', 'r', 'r']
        self.d = link_length
        self.a = [np.pi / 2, -np.pi / 2, 0, 0, 0]
        
        # default collision shapes
        self.lx = [0] * self.ndof
        self.ly = [0] * self.ndof
        self.lw = [0.05] * self.ndof
        self.ll = link_length
        self.adapt_ll = True
        

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    print("test ik 2P3R robot")
    print("--------------")
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    plt.title("2P3R Robot")
    r3 = Robot_2P3R([1, 1, 0.5, 0.5, 0.3])
    r3.plot(ax, [0.5, 0.5, 0.1, -0.1, 0.3])
    
    qr = np.linspace(0, 1, 10)
    dr = np.linspace(0.5, 1, 10)
    qt = np.array([dr, dr, qr, qr, qr]).T
    r3.plot_path_kinematics(ax, qt)
    
    print("test ik 3R robot")
    print("--------------")
    
    r4 = Robot_3R([1, 1, 0.5], [0.05, 0.05, 0.05])
    p = [1.02359743,  2.09419032,  0.88539816]
    sol2 = r4.ik(p)
    if sol2['success']:
        sol2 = sol2['q']
        fig = plt.figure()
        ax4 = fig.gca()
        plt.axis('equal')
        plt.axis([-1, 3, -1, 3])
        plt.title("3R Robot")
        r4.plot_path(ax4, sol2)
    else:
        print("ik 3R robot failed")
    
