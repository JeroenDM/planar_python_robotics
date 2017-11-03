#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import geometry_2D as geo

class Robot:
    """ serial links robot in 2D
    
    assume base at (0, 0)
    """
    
    def __init__(self, link_length, link_width):
        if len(link_length) != len(link_width):
            raise ValueError("l and d must have the same length.")
        self.ndof = len(link_length)
        self.l = link_length
        self.w = link_width
    
    def getLinkShape(self, i, xi, yi, phii):
        return geo.Rectangle(xi,
                             yi - self.w[i] / 2,
                             self.l[i],
                             self.w[i],
                             phii)
    
    def plot(self, axes_handle, q):
        p = self.fk(q)
        for i in range(self.ndof):
            recti = self.getLinkShape(i, p[i, 0], p[i, 1], p[i, 2])
            recti.plot(axes_handle)
    
    def fk(self, q):
        """ return array with all links and end effector position
        and orientation
        """
        pos = np.zeros((self.ndof, 3))
        pos[0, 2] = q[0]
        for i in range(self.ndof-1):
            dpx = self.l[i] * np.cos(pos[i, 2])
            dpy = self.l[i] * np.sin(pos[i, 2])
            pos[i+1, :2] = pos[i, :2] + [dpx, dpy]
            pos[i+1, 2] = pos[i, 2] + q[i+1]
        return pos

if __name__ == "__main__":
    
    rob = Robot([1, 1, 0.5], [0.1, 0.05, 0.05])
    
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    
    q1 = [np.pi/2, -np.pi/4, 0.1]
    pos1 = rob.fk(q1)
    print(pos1)
    rob.plot(ax, q1)