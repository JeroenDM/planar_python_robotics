#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 16:21:37 2017

@author: jeroen

Given tolerances trajectory points
p1, p2, ..., pN

For each point we can discretize the tolerane creating a meshgrid resulting
in cartesian points in space
[c11, c12, ...] , [c21, c22, ...], ..., [cN1, cN2, ...]

Now calculating the inverse kinematics for all this cartesian points we get
[ (q111, q112 ,..), (q121, q122, ...)], [ (q211, q212 ,..), (q221, q222, ...)],
... , [ (qN11, qN12 ,..), (qN21, qN22, ...)]

Now ignoring the different configurations for a moment we can represent this
in N flat arrays (but not a matrix because not every traj point has
the same number of joint solutions)
[q11, q12, q13], [q21, q22, ...], ..., [qN1, qN2, ...]
or
Q1, Q2, ..., QN (with lengths n1, n2, ..., nN)

The cost of a transitions between the joint solutions can be represented
in matrices
M1 , M2, ..., M(N-1)
with size
(n2 x n1), (n3 x n2), ..., (nN x n(N-1))

Implementation
--------------

The trajectory point with tolerances seems to me a good class
The we do actions on a list of such points which return new lists
These actions seems to be functions of a module, a more functional approach
where the objects (trajectory points) are not modified

(classed only as data structure ??)

A toleranced value can also be an object probable
(in 3D this is a toleranced frame such as given in descartes)
"""
import numpy as np

class tolValue:
    def __init__(self, nominal, lower_bound, upper_bound, samples=10):
        """ nominal does not have to be in the middle,
        it is the preffered value when we ever calculate some kind of cost
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
        """
        self.x = p[0]
        self.y = p[1]
        self.phi = p[2]
        """
        self.dim = len(pee)
        self.p = pee
        self.hasTolerance = [isinstance(pee[i], tolValue) for i in range(self.dim)]
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

def to_joint_space(cart_points, robot):
    """ we can lose the info to which cart point a joint solution
    belongs
    New data type, or use forward kinematics when needed??
    
    """
    q = []
    for p in cart_points:
        q.append(np.array(robot.ik_all(p)))
    return np.vstack(q)

if __name__ == "__main__":
    from robot import Robot
    import matplotlib.pyplot as plt
    print("some testing")
    
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    
    # create trajectory
    dx    = tolValue(1, 0.9, 1.1, samples=3)
    angle = tolValue(0.0, -0.5, 0.5, samples=5)
    
    traj = []
    N_traj = 10
    for i in range(N_traj):
        yi = 0.7 + i * 0.6 / 10
        traj.append(TrajectoryPt([dx, yi, angle]))
    
    # plot nominal trajectory
    xt = [tp.p_nominal[0] for tp in traj]
    yt = [tp.p_nominal[1] for tp in traj]
    ax.plot(xt, yt, '*')
    
    # create list with vectors containing cartesian points for every
    # trajectory point
    cart_traj = []
    for pt in traj:
        cart_traj.append(discretize(pt))
    
    
    # solve inverse kinematics for all cart points in cart_traj
    rob = Robot([1, 1, 0.5], [0.02, 0.02, 0.02])
    joint_traj = []
    for cart_vec in cart_traj:
        for cart_pt in cart_vec:
            sol = rob.ik_all_3R(cart_pt)
            if sol['success']:
                joint_traj.append(sol)
            else:
                print("no solution found for " + str(cart_pt))
#    
#    gg = discretize(pt)
#    print(gg)
#    
#    rob = Robot([1, 1, 0.5], [0.02, 0.02, 0.02])
#    qq = to_joint_space(gg, rob)
#    print(qq)
#    

#    for i in range(5):
#        rob.plot(ax, qq[i])
    