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
# test for approximate equality (for floating point types)


def inList(myarr, list_arrays):
    return next((True for elem in list_arrays if elem.size == myarr.size and np.allclose(elem, myarr)), False)


class Robot:
    """ serial links robot in 2D
    
    assume base at (0, 0)
    assume joint limits -2*pi -> 2*pi
    """ 
    def __init__(self, link_length, link_width):
        """ Robot with a rectangle for every link """
        
        if len(link_length) != len(link_width):
            raise ValueError("l and d must have the same length.")

        self.ndof = len(link_length)
        self.l = link_length
        self.w = link_width

    def plot(self, axes_handle, q, *arg, **karg):
        """ Plot the robot with joint position q """
        for recti in self.get_rectangles(q):
            recti.plot(axes_handle, *arg, **karg)

    def plot_path(self, axes_handle, qp):
        """ Plot a list of joint positions """
        
        # draw robot more transparant to the end of the path
        alpha = np.linspace(1, 0, len(qp))
        
        for i, qi in enumerate(qp):
            for rect in self.get_rectangles(qi):
                rect.plot(axes_handle, color=(0.1, 0.2, 0.5, alpha[i]))

    def check_collision(self, q, other_rectangles):
        """ Check for collision between the robot other_rectangles """
        
        for recti in self.get_rectangles(q):
            for rectj in other_rectangles:
                if recti.in_collision(rectj):
                    return True # exit if a collision is found
        return False

    def get_link_shape(self, i, xi, yi, phii):
        """ Get a rectangle of a specific robot with pose [xi, yi, phii] """
        return geom.Rectangle(xi, yi, self.l[i], self.w[i], phii)

    def get_rectangles(self, q):
        """ Get a list with rectangles for all robot links in position q """
        p = self.fk_all_links(q)
        rect = []
        for i in range(self.ndof):
            rect.append(self.get_link_shape(i, p[i, 0], p[i, 1], p[i+1, 2]))
        return rect

    def fk_all_links(self, q):
        """ Calculate position of all links, and the end effector """
        
        pos = np.zeros((self.ndof+1, 3))
        pos[0, 2] = 0
        for i in range(self.ndof):
            pos[i+1, 2] = pos[i, 2] + q[i]  # absolute orientation link i
            pos[i+1, 0] = pos[i, 0] + self.l[i] * np.cos(pos[i+1, 2])
            pos[i+1, 1] = pos[i, 1] + self.l[i] * np.sin(pos[i+1, 2])
        return pos

    def fk(self, q):
        """ Calculate end effector position and orientation """
        
        return self.fk_all_links(q)[-1]

    def ik(self, pee, q0):
        """ Solve inverse kinematic equation pee = fk(q)
        
        Parameters
        ----------
        pee : ndarray
            array or list containing end-effector position and orientation
        q0 : ndarray
            array or list containing an initial guess for the joint solution
        """
        
        sol = root(lambda x: self.fk(x) - pee, q0)

        if sol['success']:
            return {'success': True, 'q': geom.normalize_angle(sol.x)}
        else:
            return {'success': False}
            #raise RuntimeError("Inverse kinematics did not converge")
        
    def ik_all(self, pee, N=4):
        """ Get many ik solutions by using many different initial q's """
        
        # create random joint angles
        q_range = np.random.rand(N) * 2 * np.pi - np.pi
        qq = [q_range] * self.ndof # TODO: use different random q's
        grid = np.meshgrid(*qq)
        grid = [ grid[i].flatten() for i in range(self.ndof) ]
        grid = np.array(grid).T
        
        # run numeric ik with different initial conditions
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
    
    def ik_analytic_3R(self, p):
        """ Analytic inverse kinematics for 3 link robot """
        
        # define variables for readability
        l1, l2, l3 = (self.l[0], self.l[1], self.l[2])
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

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    rob = Robot([1, 1, 0.5], [0.1, 0.05, 0.05])
    
    print("test vector q1")
    q1 = [np.pi/2, -np.pi/4, 0.1]
    print(q1)
    pos1 = rob.fk_all_links(q1)
    pos1b = rob.fk(q1)
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
    sol2 = rob.ik_analytic_3R(p)
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
    