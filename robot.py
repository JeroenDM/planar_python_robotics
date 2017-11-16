#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import numpy as np
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
    
    def getRectangles(self, q):
        p = self.fk(q)
        rect = []
        for i in range(self.ndof):
            rect.append(self.getLinkShape(i, p[i, 0], p[i, 1], p[i+1, 2]))
        return rect
    
    def plot(self, axes_handle, q, *arg):
        for recti in self.getRectangles(q):
            recti.plot(axes_handle, *arg)
    
    def check_collision(self, q, col_rect):
        for recti in self.getRectangles(q):
            for rectj in col_rect:
                if recti.inCollision(rectj):
                    return True
        return False
    
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
            return {'success': True, 'q': geom.fixAngle(sol.x)}
        else:
            return {'success': False}
            #raise RuntimeError("Inverse kinematics did not converge")
        
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
        success = False
        for i in range(len(grid)):
            sol = self.ik(pee, grid[i])
            if sol['success']:
                if not inList(sol['q'], q_list):
                    q_list.append(sol['q'])
                    success = True
        if success:
            return {'success': True, 'q': q_list}
        else:
            return {'success': False}
    
    def ik_all_3R(self, p):
        l1 = self.l[0]; l2 = self.l[1]; l3 = self.l[2];
        # define variables for readability
        x, y, phi = (p[0], p[1], p[2])
        q_up = [0, 0, 0]
        q_do = [0, 0, 0]
        
        reachable = False
        
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
#            check_up = self.checkJointLimits(q_up)
#            check_do = self.checkJointLimits(q_do)
#            if check_up & check_do:
#                return {'success': True, 'q': [q_up, q_do]}
#            elif check_up:
#                return {'success': True, 'q': [q_up]}
#            elif check_do:
#                return {'success': True, 'q': [q_do]}
#            else:
#                return {'success': False, 'info': "joint limits"}
            return {'success': True, 'q': [q_up, q_do]}
        else:
            return {'success': False, 'info': "unreachable"}
    
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
    sol = rob.ik(p, q_init)
    if sol['success']:
        print(sol['q'])
        rob.plot(ax, sol['q'], 'g')
    else:
        print("ik failed")
    
    print("test ik all")
    sol2 = rob.ik_all(p)
    if sol2['success']:
        sol2 = sol2['q']
        fig = plt.figure()
        ax2 = fig.gca()
        plt.axis('equal')
        plt.axis([-1, 3, -1, 3])
        for qi in sol2:
            print(qi)
            rob.plot(ax2, qi, 'b')
    else:
        print("ik_all failed")
        
    print("test ik all 3R")
    sol2 = rob.ik_all_3R(p)
    if sol2['success']:
        sol2 = sol2['q']
        fig = plt.figure()
        ax2 = fig.gca()
        plt.axis('equal')
        plt.axis([-1, 3, -1, 3])
        for qi in sol2:
            print(qi)
            rob.plot(ax2, qi, 'b')
    else:
        print("ik_all failed")
    