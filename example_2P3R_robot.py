#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 19 12:37:49 2018

@author: jeroen

Compiler flag -02 in c++ made it faster!
7.5 to 1.2 seconds for a 2P3R example with ik_sample=6
"""

""" code block 1 """
import matplotlib.pyplot as plt

from ppr.path import TolerancedNumber, TrajectoryPt, plot_path

# create tolerances for x-position and orientation
dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)

# create a list with path points
path = []
n_path = 12
for i in range(n_path):
   yi = 0.7 + i * 0.6 / n_path
   path.append(TrajectoryPt([dx, yi, angle]))

""" code block 2 """
from example_robots import Robot_2P3R

# create a robot with a link lengths and link widths
# (The width is used to create rectangles around the links as a collision model)
robot1 = Robot_2P3R([1, 1, 0.5, 0.5, 0.3])

# plot the robot in some configuration
fig2, ax2 = plt.subplots()
plt.title("The Robot")
ax2.axis('equal')
robot1.plot(ax2, [0.5, 0.5, 1.3, -0.8, 1.4], 'k')
plot_path(ax2, path)
#plt.savefig("image/example_robot.png")

from ppr.path import cart_to_joint_d

Q = cart_to_joint_d(robot1, path, ik_sample = 6)

fig3, ax3 = plt.subplots()
robot1.plot_path_kinematics(ax3, Q[0][1:30])
print(Q[0].shape)

from ppr_cpp.graph_cpp import get_shortest_path

res = get_shortest_path(Q)

# MMM, UNITS of angle and distance are mixed
# convert it to actual motor rotation??


#%%
#if res['success']:
#    shortest_path_js = res['path']
#    fig4, ax4 = plt.subplots()
#    plt.title("graph search solution")
#    ax4.axis('equal')
#    robot1.plot_path(ax4, shortest_path_js)
#    plot_path(ax4, path, show_tolerance=False)
#
##%%
#
#j1 = [p[0] for p in shortest_path_js]
#print(j1)    