#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan  2 11:56:43 2018

@author: jeroen
"""

import matplotlib.pyplot as plt

from ppr.path import TolerancedNumber, TrajectoryPt, plot_path, discretize

# create tolerances for x-position and orientation
dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)

# create a list with path points
path = []
n_path = 12
for i in range(n_path):
   yi = 0.7 + i * 0.6 / n_path
   path.append(TrajectoryPt([dx, yi, angle]))
   
path2 = []
n_path = 12
for i in range(n_path):
   yi = 0.7 + i * 0.6 / n_path
   path2.append(TrajectoryPt([0.9, yi, 0.0]))

# look at the created path
fig1, ax1 = plt.subplots()
plt.title("The path")
ax1.axis('equal')
plot_path(ax1, path)

from example_robots import Robot_3R

# create a robot with a link lengths and link widths
# (The width is used to create rectangles around the links as a collision model)
robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])

# plot the robot in some configuration
fig2, ax2 = plt.subplots()
plt.title("The Robot")
ax2.axis('equal')
robot1.plot(ax2, [1.3, -0.8, 1.4], 'k')

from ppr.path import cart_to_joint
from ppr.geometry import Rectangle
from ppr.scene import plot_scene

# create list collision objects (Rectangles)
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

path_js = cart_to_joint(robot1, path2, check_collision = True, scene = sc1)

import numpy as np

wp = np.vstack(path_js)



fig3, ax3 = plt.subplots()
plt.title("The joint solutions")
ax3.axis('equal')
robot1.plot_path_kinematics(ax3, wp)
plot_scene(ax3, sc1, 'r')