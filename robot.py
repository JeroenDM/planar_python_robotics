#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import geometry as geom
from scipy.optimize import root

# https://stackoverflow.com/questions/23979146/check-if-numpy-array-is-in-list-of-numpy-arrays
#test for approximate equality (for floating point types)
def inList(myarr, list_arrays):
    return next((True for elem in list_arrays if elem.size == myarr.size and np.allclose(elem, myarr)), False)

class Robot:
    """ serial links robot in 2D
    
    assume base at (0, 0)
    assume joint limits -pi -> pi
    """
    
    def __init__(self, link_length, link_width):
        if len(link_length) != len(link_width):
            raise ValueError("l and d must have the same length.")
        self.ndof = len(link_length)
        self.l = link_length
        self.w = link_width
    
    def getLinkShape(self, i, xi, yi, phii):
        return geom.Rectangle(xi, yi, self.l[i], self.w[i], phii)
    
    def plot(self, axes_handle, q, *arg):
        p = self.fk(q)
        for i in range(self.ndof):
            recti = self.getLinkShape(i, p[i, 0], p[i, 1], p[i+1, 2])
            recti.plot(axes_handle, *arg)
    
    def fk(self, q):
        """ return array with all links and end effector position
        and orientation
        
        Base frame 0
        Link frame 1 -> ndof
        """
        pos = np.zeros((self.ndof+1, 3))
        pos[0, 2] = 0
        for i in range(self.ndof):
            pos[i+1, 2] = pos[i, 2] + q[i] # absolute orientation link i
            pos[i+1, 0] = pos[i, 0] + self.l[i] * np.cos(pos[i+1, 2])
            pos[i+1, 1] = pos[i, 1] + self.l[i] * np.sin(pos[i+1, 2])
        return pos
    
    def fk_short(self, q):
        """ only return end effector position and orientation
        """
        return self.fk(q)[-1]
#        Tee = np.eye(3)
#        for i in range(self.ndof):
#            Ti = self.transform(i, q[i])
#            Tee = np.dot(Tee, Ti)
#        return Tee[:2, 2]
        
    def ik(self, pee, q0):
        """ solve inverse kinematic equations
        
        pee = fk(q)
        Parameters
        ----------
        pee : ndarray
            array or list containing end-effector position and orientation
        q0 : ndarray
            array or list containing an initial guess for the joint solution
        """
        
        sol = root(lambda x: self.fk_short(x) - pee, q0)
        if sol['success']:
            #print(sol)
            #return sol.x
            return geom.fixAngle(sol.x)
        else:
            raise RuntimeError("Inverse kinematics did not converge")
        
    def ik_all(self, pee, N=4):
        """ try to calculate all different ik solutions by using
        different random q0 ndof^N times ik calculations !!
        """
        q_range = np.random.rand(N) * 2 * np.pi - np.pi
        qq = [q_range] * self.ndof
        grid = np.meshgrid(*qq)
        grid = [ grid[i].flatten() for i in range(self.ndof) ]
        grid = np.array(grid).T
#        print(grid.shape)
        
        q_list = []
        for i in range(len(grid)):
            qi = self.ik(pee, grid[i])
            if not inList(qi, q_list):
                q_list.append(qi)
#            print(qi)
        return q_list
    
    def transform(self, i, qi):
        """ Get the homogeneous transformation matrix for joint i
        
        The transformation matrix in function of joint movement i expresses
        frame i relative to i-1
        
        Parameters
        ----------
        i : int
            joint number from 0 to ndof-1
        
        Returns
        -------
        numpy.ndarray
            Homogenous transformation matrix for joint i
            expressing frame i relative to i-1
        """
        return geom.transform(qi, self.l[i])

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np
    rob = Robot([1, 1, 0.5], [0.1, 0.05, 0.05])
    
    print("test vector q1")
    q1 = [np.pi/2, -np.pi/4, 0.1]
    print(q1)
    pos1 = rob.fk(q1)
    pos1b = rob.fk_short(q1)
    print("caluclate all fk")
    print(pos1)
    print("short fk")
    print(pos1b)
    
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    rob.plot(ax, q1, 'b')
    
    print("test ik")
    print("-------------")
    q_init = [-0.2, 0.1, 0.1]
    p = [1.02359743,  2.09419032,  0.88539816]
    q_sol = rob.ik(p, q_init)
    print(q_sol)
    rob.plot(ax, q_sol, 'g')
    
    print("test ik all")
    sol2 = rob.ik_all(p)
    
    fig = plt.figure()
    ax2 = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    for qi in sol2:
        print(qi)
        rob.plot(ax2, qi, 'b')
    