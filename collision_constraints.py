#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 09:49:52 2018

@author: jeroen
"""

import matplotlib.pyplot as plt
from example_robots import Robot_3R
#from ppr.cpp.geometry_cpp import Rectangle
from ppr.geometry import Rectangle

from ppr.scene import plot_scene

# create a robot with a link lengths and link widths
# (The width is used to create rectangles around the links as a collision model)
robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])
# create list collision objects (Rectangles
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, 0),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

# plot the robot in some configuration
fig2, ax2 = plt.subplots()
plt.title("The Robot")
ax2.axis('equal')
robot1.plot(ax2, [1.3, -0.8, 1.4], 'k')
plot_scene(ax2, sc1, 'r')



A = sc1[0].get_normals()
print(A)