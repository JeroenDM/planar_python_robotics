#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 30 14:25:24 2017

@author: jeroen
"""

import numpy as np
from numpy.linalg import norm

""" function I have no class for """
def rotation(angle):
    return np.array( [[np.cos(angle),  -np.sin(angle)],
                     [np.sin(angle),  np.cos(angle)]]
                   )

def transform(angle, distance):
    R = rotation(angle)
    T = np.zeros((3, 3))
    T[:2, :2] = R
    T[0, 2] = distance * np.cos(angle)
    T[1, 2] = distance * np.sin(angle)
    return T

def fixAngle(x):
    """ for an input or input vector rescale an angle
        to fit in the interval (0 to pi or) -p/2 to pi/2
    """
    # define constants
    PI = np.pi
    LO = -PI
    UP = PI
    
    # make sure it works for scalar and vectors
    if np.isscalar(x):
        x = np.array([x])
        
    # create new scaled vector
    y = x.copy()
    for i in range(len(x)):
        while y[i] > UP:
            y[i] -= 2 * PI
        while y[i] < LO:
            y[i] += 2 * PI
    return y

def rotate(point, angle):
    """ Rotate a point around the origin
    
    given p = [x, y], multiply by the rotation matrix R(angle)
    """
    R = np.array([[np.cos(angle),  -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    p = np.array(point)
    return np.dot(R, p)

""" classes """
class Rectangle:
    """ rectangles to represent robot link    
    """
    def __init__(self, x, y, dx, dy, angle):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.a = angle
        self.R = rotation(angle)
        self.p = self._getPoints()
    
    def _getPoints(self):
        p = np.zeros((4, 2))
        p[0, 0] = self.x
        p[0, 1] = self.y
        p[1, :] = np.dot(self.R, [self.dx, 0 ]) + p[0, :]
        p[2, :] = np.dot(self.R, [self.dx, self.dy]) + p[0, :]
        p[3, :] = np.dot(self.R, [0 ,      self.dy]) + p[0, :]
        return p
    
    def rotate(self, angle):
        self.a += angle
        self.R = rotation(self.a)
        self.p = self._getPoints()
        
    def translate(self, trans_x, trans_y):
        self.x += trans_x
        self.y += trans_y
        self.p = self._getPoints()
        
    def moveToOrigin(self):
        self.x = 0
        self.y = 0
        self.p = self._getPoints()
    
    def getNormals(self):
        p = self.p
        n = np.zeros((4, 2))
        n[0, 0] =   p[1, 1] - p[0, 1]
        n[0, 1] = -(p[1, 0] - p[0, 0])
        n[0, :] = n[0, :] / norm(n[0, :])
        Rtemp = rotation(np.pi/2)
        n[1, :] = np.dot(Rtemp, n[0, :])
        n[2, :] = np.dot(Rtemp, n[1, :])
        n[3, :] = np.dot(Rtemp, n[2, :])
        return n
    
    def project(self, direction):
        p = self.p
        angle = -np.arctan2(direction[1], direction[0])
        px = np.zeros(4)
        for i in range(4):
            px[i] = rotate(p[i], angle)[0]
        return px
    
    def inCollision(self, rect2, tol=1e-9):
        n1 = self.getNormals()
        n2 = rect2.getNormals()
        n_all = np.vstack((n1, n2))
        col = True
        i = 0
        while col and i < 8:
            pr1 = self.project(n_all[i])
            pr2 = rect2.project(n_all[i])
            if (( max(pr1) + tol < min(pr2) ) or ( min(pr1) > max(pr2) + tol )):
                col = False
            i += 1
                
        return col
    
    def plot(self, ax, *arg, **karg):
        p = self.p
        p = np.vstack((p, p[0]))
        ax.plot(p[:, 0], p[:, 1], *arg, **karg)


if __name__ == "__main__":
    import  matplotlib.pyplot as plt
    
    # test fix angle
    x_test = np.random.rand(20)*16 - 8
    x_fix = fixAngle(x_test)
    px = [np.cos(x_test), np.cos(x_fix)]
    py = [np.sin(x_test), np.sin(x_fix)]
    print(x_test)
    print(x_fix)
    
    plt.figure()
    plt.axis('equal')
    plt.plot(px[0], py[0], '*', px[1], py[1], '.')
    
    rect1 = Rectangle(1, 2, 2, 2, np.pi/4)
    rect2 = Rectangle(1, 1, 3, 2, -np.pi/6)
    
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    rect1.plot(ax, 'g.-')
    rect2.plot(ax, '.-', color=(0.1, 0.1, 0.1, 0.5))
    
    px = rect1.project([1, 0])
    py = np.zeros(4)
    ax.plot(px, py, 'r*')
    
    res = rect1.inCollision(rect2)
    print("Do they collide: " + str(res))
    
    # test random set of rectangles
    fig2 = plt.figure(2)
    ax2 = fig2.gca()
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    nr = 20
    r_pos = np.random.rand(nr, 2) * 10 - 5
    r_sha = np.random.rand(nr, 2) * 6 - 3
    r_ang = np.random.rand(nr) * np.pi - np.pi/2
    rectangles = []
    for i in range(nr):
        rectangles.append(Rectangle(r_pos[i, 0], r_pos[i, 1],
                                    r_sha[i, 0], r_sha[i, 1],
                                    r_ang[i]))
    for rect_a in rectangles:
        rect_a_collides = False
        for rect_b in rectangles:
            if rect_a != rect_b:
                if rect_a.inCollision(rect_b):
                    rect_a_collides = True
                    break
        if rect_a_collides:
            rect_a.plot(ax2, 'r')
        else:
            rect_a.plot(ax2, 'g')
                