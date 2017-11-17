#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 12:29:32 2017

@author: jeroen
"""

import numpy as np

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
                print("no solution found for " + str(cart_pt))
        joint_traj.append(np.array(qi))
    return joint_traj
    
if __name__ == "__main__":   
    import matplotlib.pyplot as plt
    from robot import Robot
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

    # test robot
    robot1 = Robot([1, 1, 0.5], [0.02, 0.02, 0.02])
    
    jt = cart_to_joint(robot1, traj)
    
    print(len(traj), len(jt))
    for i in range(len(jt)):
        print(jt[i].shape)