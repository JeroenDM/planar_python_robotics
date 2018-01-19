#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 12:29:32 2017

@author: jeroen
"""

import numpy as np
from matplotlib.patches import Wedge

class TolerancedNumber:
    def __init__(self, nominal, lower_bound, upper_bound, samples=10):
        """ Create a range of possible numbers
        
        Nominal does not have to be in the middle,
        it is the preffered value when we ever calculate some kind of cost.
        """
        if nominal < lower_bound or nominal > upper_bound:
            raise ValueError("nominal value must respect the bounds")
        self.n = nominal
        self.u = upper_bound
        self.l = lower_bound
        self.sam = samples
        self.range = np.linspace(self.l, self.u, self.sam)
    
# adding a function can be better memory wise
#    def getRange(self):
#        return np.linspace(self.l, self.u, self.sam)


class TrajectoryPt:
    def __init__(self, pee):
        """ Create a trajectory point expressed in cartesian space
        
        pee = [x_position, y_position, angle last joint with x axis]
        """
        self.dim = len(pee)
        self.p = pee
        self.hasTolerance = [isinstance(pee[i], TolerancedNumber) for i in range(self.dim)]
        self.p_nominal = []
        for i in range(self.dim):
            if self.hasTolerance[i]:
                self.p_nominal.append(self.p[i].n)
            else:
                self.p_nominal.append(self.p[i])
        self.timing = 0.1 # with respect to previous point
    
    def plot(self, axes_handle, show_tolerance):
        pn = self.p_nominal
        axes_handle.plot(pn[0], pn[1], 'k*')
        if show_tolerance:
            if self.hasTolerance[0]:
                do = -self.p[0].l + pn[0]
                du =  self.p[0].u - pn[0]
                axes_handle.errorbar(pn[0], pn[1], xerr=[[do], [du]], color=(0.5, 0.5, 0.5))
            if self.hasTolerance[1]:
                do = -self.p[1].l + pn[1]
                du =  self.p[1].u - pn[1]
                axes_handle.errorbar(pn[0], pn[1], yerr=[[do], [du]], color=(0.5, 0.5, 0.5))
            if self.hasTolerance[2]:
                radius = 0.1
                do = self.p[2].l * 180 / np.pi
                du = self.p[2].u * 180 / np.pi
                arc = Wedge((pn[0], pn[1]), radius, do, du, facecolor=(0.5, 0.5, 0.5, 0.5))
                axes_handle.add_patch(arc)
    

def plot_path(axes_handle, path, show_tolerance=True):
    for pt in path:
        pt.plot(axes_handle, show_tolerance)
   
def discretize(pt):
    r = []
    for i in range(pt.dim):
        if pt.hasTolerance[i]:
            r.append(pt.p[i].range)
        else:
            r.append(pt.p[i])
    grid = np.meshgrid(*r)
    grid = [ grid[i].flatten() for i in range(pt.dim) ]
    grid = np.array(grid).T
    
    return grid

def cart_to_joint(robot, traj_points, check_collision = False, scene=None):
    # get sampled version of trajectory points
    cart_traj = []
    for pt in traj_points:
        cart_traj.append(discretize(pt))
    
    if check_collision:
        if scene == None:
            raise ValueError("scene is needed for collision checking")
    # solve inverse kinematics for every samples traj point
    joint_traj = []
    for cart_vec in cart_traj:
        qi = []
        for cart_pt in cart_vec:
            sol = robot.ik(cart_pt)
            if sol['success']:
                for qsol in sol['q']:
                    if check_collision:
                        if not robot.check_collision(qsol, scene):
                            qi.append(qsol)
                    else:
                        qi.append(qsol)
            else:
                print("[ppr.path.py] no collision free solution found for " + str(cart_pt))
        joint_traj.append(np.array(qi))
    return joint_traj

def cart_to_joint_d(robot, traj_points, check_collision = False, scene=None, ik_sample=5):
    # get sampled version of trajectory points
    cart_traj = []
    for pt in traj_points:
        cart_traj.append(discretize(pt))
    
    if check_collision:
        if scene == None:
            raise ValueError("scene is needed for collision checking")
    # solve inverse kinematics for every samples traj point
    joint_traj = []
    for cart_vec in cart_traj:
        qi = []
        for cart_pt in cart_vec:
            sol = robot.ik_discrete(cart_pt, n_sample=ik_sample)
#            print("Ik solution found for traj point " + str(cart_pt))
#            print(len(sol))
            if sol['success']:
                for qsol in sol['q']:
                    if check_collision:
                        if not robot.check_collision(qsol, scene):
                            qi.append(qsol)
                    else:
                        qi.append(qsol)
            else:
                print("no solution found for " + str(cart_pt))
        joint_traj.append(np.array(qi))
    return joint_traj
    
if __name__ == "__main__":
    print("-----test path.py-----")
    import matplotlib.pyplot as plt
#    from robot import Robot
    # create trajectory
    dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
    angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)
    
    traj = []
    N_traj = 10
    for i in range(N_traj):
        yi = 0.7 + i * 0.6 / 10
        traj.append(TrajectoryPt([dx, yi, angle]))
    
    # plot nominal trajectory
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    xt = [tp.p_nominal[0] for tp in traj]
    yt = [tp.p_nominal[1] for tp in traj]
    ax.plot(xt, yt, '*')

    # TODO implement ik for general robot
    # or add example robots to package
    # test robot
#    r1 = Robot(['r', 'r', 'r'], [1, 1, 0.5], [0, 0, 0])
    
#    jt = cart_to_joint(r1, traj)
#    
#    print(len(traj), len(jt))
#    for i in range(len(jt)):
#        print(jt[i].shape)