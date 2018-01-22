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

""" add collision objects """
from ppr.cpp.geometry_cpp import Rectangle
#from ppr.geometry import Rectangle
from ppr.scene import plot_scene

# create list collision objects (Rectangles
sc1 = [Rectangle(0.0, 0.4, 0.1, 0.2, -0.3),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

# plot the robot in some configuration
fig2, ax2 = plt.subplots()
plt.title("The Robot")
ax2.axis('equal')
robot1.plot(ax2, [0.5, 0.5, 1.3, -0.8, 1.4], 'k')
plot_path(ax2, path)
plot_scene(ax2, sc1, 'r')
#plt.savefig("image/example_robot.png")

""" graph search """
from ppr.path import cart_to_joint_d

Q = cart_to_joint_d(robot1, path, check_collision=True, scene=sc1, ik_sample = 6)

fig3, ax3 = plt.subplots()
robot1.plot_path_kinematics(ax3, Q[0][1:30])
print(Q[0].shape)

from ppr.cpp.graph_cpp import get_shortest_path

res = get_shortest_path(Q)

# MMM, UNITS of angle and distance are mixed
# convert it to actual motor rotation??


#%%
if res['success']:
    shortest_path_js = res['path']
    fig4, ax4 = plt.subplots()
    plt.title("graph search solution")
    ax4.axis('equal')
    robot1.plot_path(ax4, shortest_path_js)
    plot_path(ax4, path, show_tolerance=False)
    plot_scene(ax4, sc1, 'r')

#%%

#j1 = [p[0] for p in shortest_path_js]
#print(j1)    
    
"""fictional code block 5 """
from ppr.optimize import get_optimal_trajectory, q_derivatives

#robot1.add_joint_speed_limits([-0.5, -0.5, -0.5], [0.5, 0.5, 0.5])
#robot1.add_joint_acceleration_limits([-0.5, -0.5, -0.5], [0.5, 0.5, 0.5])
# set link mass, cg position and mass moment of inertia.
robot1.set_link_inertia([1, 1, 1, 0.5, 0.5],
                        [0.5, 0.5, 0.25, 0.25, 0.15],
                        [0.05, 0.05, 0.05, 0.05, 0.05])

qs, dqs, ddqs = get_optimal_trajectory(robot1, path, shortest_path_js)

fig5, ax5 = plt.subplots()
plt.title("Optimized solution")
ax5.axis('equal')
robot1.plot_path(ax5, qs)
plot_path(ax5, path, show_tolerance=False)
plot_scene(ax5, sc1, 'r')