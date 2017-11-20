#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import numpy as np
from ppr.robot import Robot
from ppr.path import TolerancedNumber
from ppr.geometry import rotation

# TODO use contructures of base class
# TODO all in and output numpy arrays ??
# TODO handle edge case where some of the joint angles are pi or -pi

class Robot_3R(Robot):
    def __init__(self, link_length, link_width=None):
        """ Robot with a rectangle for every link """
        
        # all input lists must have the same length
        self.ndof = 3
        self.jt = ['r', 'r', 'r']
        self.d = link_length
        self.a = [0, 0, 0]
        
        # default collision shapes
        self.lx = [0] * self.ndof
        self.ly = [0] * self.ndof
        if link_width == None:
            self.lw = [0.02, 0.02, 0.02]
        else:
            self.lw = link_width
        self.ll = link_length
        self.adapt_ll = False
        self.base = [0, 0, 0]
    
    def ik(self, p, tol=1e-12):
        """ Analytic inverse kinematics for 3 link robot """
        # tol = tolerance needed for boundary cases
        # base transform only translation for now TODO
        p = np.array(p)
        p[:2] = p[:2] - self.base[:2]
        
        # define variables for readability
        l1, l2, l3 = (self.d[0], self.d[1], self.d[2])
        x, y, phi = (p[0], p[1], p[2])
        
        # initialize output
        q_up = [0, 0, 0]
        q_do = [0, 0, 0]
        reachable = False
        
        # start calculations
        if (l1 + l2 + l3) >= np.sqrt(x**2 + y**2):
            # coordinates of end point second link (w)
            pwx = x - l3 * np.cos(phi)
            pwy = y - l3 * np.sin(phi)
            rws = pwx**2 + pwy**2
            if (l1 + l2) >= np.sqrt(rws):
                reachable = True
                # calculate q2
                c2 = (rws - l1**2 - l2**2) / (2*l1*l2)
                # if c2 exactly 1, it can be a little bit bigger at this point
                # because of numerical error, then rescale
                # TODO far from all edge cases are catched
                if abs(c2 - 1) < tol:
                    c2 = np.sign(c2) * 1.0
                    s2 = 0.0
                    q_up[1] = np.arctan2(0, c2) # elbow up
                    q_do[1] = -np.arctan2(0, c2) # elbow down
                else:
                    s2 = np.sqrt(1 - c2**2)
                    q_up[1] = np.arctan2(s2, c2) # elbow up
                    q_do[1] = np.arctan2(-s2, c2) # elbow down
                # calculate q1
                temp = (l1 + l2 * c2)
                s1_up = (temp * pwy - l2 * s2 * pwx) / rws
                c1_up = (temp * pwx + l2 * s2 * pwy) / rws
                s1_do = (temp * pwy + l2 * s2 * pwx) / rws
                c1_do = (temp * pwx - l2 * s2 * pwy) / rws
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
        self.base = [0, 0, 0]
        
        # create 3R robot for inverse kinematics
        self.sub_robot = Robot_3R(link_length[2:])

    def ik(self, p, q_fixed = [0, 0]):
        """ Because ndof > len(p), two joints are fixed for ik """
        
        q_base = [0, 0, 0]
        q_base[0] = q_fixed[1]
        q_base[1] = q_fixed[0]
        self.sub_robot.set_base_pose(q_base)
        sub_sol = self.sub_robot.ik(p)
        if sub_sol['success']:
            # add fixed joints to solution
            q_sol = []
            for qi in sub_sol['q']:
                q_sol.append([*q_fixed, *qi])
            sub_sol['q'] = q_sol
        return sub_sol

    def ik_discrete(self, p, n_sample = 5):
        q1 = TolerancedNumber(0.5, 0, 1, samples=n_sample)
        q2 = TolerancedNumber(0.5, 0, 1.5, samples=n_sample)
        grid = np.meshgrid(q1.range, q2.range)
        grid = [ grid[i].flatten() for i in range(2) ]
        grid = np.array(grid).T
        
        q_sol = []
        for qf in grid:
            s = self.ik(p, q_fixed=qf)
            if s['success']:
                for qi in s['q']:
                    q_sol.append(qi)
        if len(q_sol) > 0:
            return {'success': True, 'q': q_sol}
        else:
            return {'success' : False, 'info': "unreachable"}

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    
    print("test ik 2P3R robot")
    print("--------------")
#    fig = plt.figure()
#    ax = fig.gca()
#    plt.axis('equal')
#    plt.axis([-1, 3, -1, 3])
#    plt.title("2P3R Robot")
    r3 = Robot_2P3R([1, 1, 0.5, 0.5, 0.3])
#    r3.plot(ax, [0.5, 0.5, 0.1, -0.1, 0.3])
#    
#    qr = np.linspace(0, 1, 10)
#    dr = np.linspace(0.5, 1, 10)
#    qt = np.array([dr, dr, qr, qr, qr]).T
#    r3.plot_path_kinematics(ax, qt)
    
    q_test = [0.5, 0.3, 0.2, -0.1, 1.1]
    pee = r3.fk_all_links(q_test)[-1]
#    print("test fk 2p3r:")
#    print(q_test)
#    print(pee)
#    
#    sol = r3.ik(pee, q_fixed=q_test[:2])
#    if sol['success']:
#        sol = sol['q']
#        fig = plt.figure()
#        ax2 = fig.gca()
#        plt.axis('equal')
#        plt.axis([-1, 3, -1, 3])
#        plt.title("ik 2P3R Robot")
#        r3.plot_path_kinematics(ax2, sol)
#    else:
#        print("ik 3R robot failed")
    
#    sol = r3.ik(pee, q_fixed=[ 0.6, 1.3])
#    if sol['success']:
#        sol = sol['q']
#        r3.plot_path_kinematics(ax2, sol)
#    else:
#        print("ik 3R robot failed")
    
    print("test discrete ik")
    sol = r3.ik_discrete(pee)
    
    if sol['success']:
        q_sol = sol['q']
        fig = plt.figure()
        ax3 = fig.gca()
        plt.axis('equal')
    #    plt.axis([-1, 3, -1, 3])
        plt.title("ik discrete 2P3R Robot")
        r3.plot_path_kinematics(ax3, q_sol)
    
#    print("test ik 3R robot")
#    print("--------------")
#    
#    r4 = Robot_3R([1, 1, 0.5], [0.05, 0.05, 0.05])
#    r4.set_base_pose([0.5, 0.3, 0.0])
#    p = [1.02359743,  2.09419032,  0.88539816]
#    sol2 = r4.ik(p)
#    if sol2['success']:
#        sol2 = sol2['q']
#        fig = plt.figure()
#        ax4 = fig.gca()
#        plt.axis('equal')
#        plt.axis([-1, 3, -1, 3])
#        plt.title("3R Robot")
#        ax4.plot(p[0], p[1], '*')
#        r4.plot_path_kinematics(ax4, sol2)
#    else:
#        print("ik 3R robot failed")