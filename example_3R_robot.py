#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Code for example on wiki

@author: jeroen
"""

""" code block 1 """
import matplotlib.pyplot as plt

from ppr.path import TolerancedNumber, TrajectoryPt, plot_path

# create tolerances for x-position and orientation
dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)

# create a list with path points
path = []
n_path = 10
for i in range(n_path):
   yi = 0.7 + i * 0.6 / 10
   path.append(TrajectoryPt([dx, yi, angle]))

# look at the created path
fig1, ax1 = plt.subplots()
plt.title("The path")
ax1.axis('equal')
plot_path(ax1, path)
plt.savefig("image/example_path.png")

""" code block 2 """
from example_robots import Robot_3R

# create a robot with a link lengths and link widths
# (The width is used to create rectangles around the links as a collision model)
robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])

# plot the robot in some configuration
fig2, ax2 = plt.subplots()
plt.title("The Robot")
ax2.axis('equal')
robot1.plot(ax2, [1.3, -0.8, 1.4], 'k')
plt.savefig("image/example_robot.png")

""" code block 3 """
from ppr.path import cart_to_joint
from ppr.geometry import Rectangle
from ppr.scene import plot_scene

# create list collision objects (Rectangles)
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

# calculate all possible (collision free) joint configurations
# for each path point
path_js = cart_to_joint(robot1, path, check_collision = True, scene = sc1)

fig3, ax3 = plt.subplots()
plt.title("The joint solutions")
ax3.axis('equal')
robot1.plot_path_kinematics(ax3, path_js[0])
plot_scene(ax3, sc1, 'r')
plt.savefig("image/example_joint_solutions.png")

""" code block 4 """
from ppr.graph import get_shortest_path

# find the best sequence of joint solutions in path_js
# currently total joint movement is minimized by default
path_length, shortest_path_js = get_shortest_path(path_js)

fig4, ax4 = plt.subplots()
plt.title("The first solution")
ax4.axis('equal')
robot1.plot_path(ax4, shortest_path_js)
plot_path(ax4, path, show_tolerance=False)
plot_scene(ax4, sc1, 'r')
plt.savefig("image/example_first_solution.png")

"""fictional code block 5 """

#from ppr.optimize import get_optimal_trajectory
#
#robot1.add_joint_speed_limits([-0.5, -0.5, -0.5], [0.5, 0.5, 0.5])
#robot1.add_joint_acceleration_limits([-0.5, -0.5, -0.5], [0.5, 0.5, 0.5])
#
#traj, info = get_optimal_trajectory(robot1, path, path_js_init=shortest_path_js)
