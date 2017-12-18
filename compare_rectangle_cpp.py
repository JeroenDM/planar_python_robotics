#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Dec 18 15:04:37 2017

@author: jeroen
"""

from ppr.geometry import Rectangle

#import ppr_cpp.geometry as gm
#class Rectangle(gm.Rectangle):
#    """Wrap the cpp class for visualisation"""
#
#    def plot(self, axes_handle, *arg, **karg):
#        """Plot the rectangle on axes"""
#        p = self.get_plot_points()
#        p = np.vstack((p, p[0]))
#        axes_handle.plot(p[:, 0], p[:, 1], *arg, **karg)

import numpy as np

def create_random_rectangles(nr=20):
    r_pos = np.random.rand(nr, 2) * 10 - 5
    r_sha = np.random.rand(nr, 2) * 20 / nr
    r_ang = np.random.rand(nr) * np.pi - np.pi/2
    rectangles = []
    for i in range(nr):
        rectangles.append(Rectangle(r_pos[i, 0], r_pos[i, 1],
                                    r_sha[i, 0], r_sha[i, 1],
                                    r_ang[i]))
    return rectangles

def check_collision(rects):
    col = np.zeros(len(rects), dtype=bool)
    for i, rect_a in enumerate(rects):
        col[i] = False
        for rect_b in rects:
            if rect_a != rect_b:
                if rect_a.in_collision(rect_b):
                    col[i] = True
                    break
    return col

#=====================
# MAIN block
#=====================

import matplotlib.pyplot as plt

fig, ax = plt.subplots()
plt.axis('equal')
plt.axis([-10, 10, -10, 10])

N = 50
for k in range(10):
    rectangles = create_random_rectangles(N)
    cols = check_collision(rectangles)

for i in range(N):
    if cols[i]:
        rectangles[i].plot(ax, 'r')
    else:
        rectangles[i].plot(ax, 'g')