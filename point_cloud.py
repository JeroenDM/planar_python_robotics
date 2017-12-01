#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Code for example on wiki

@author: jeroen
"""

""" code block 1 """
import matplotlib.pyplot as plt
import numpy as np

from example_robots import Robot_3R
from ppr.path import TolerancedNumber, TrajectoryPt, plot_path
from ppr.path import cart_to_joint
from ppr.geometry import Rectangle
from ppr.scene import plot_scene
from ppr.graph import get_shortest_path


# create tolerances for x-position and orientation
dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
angle = TolerancedNumber(0.0, -1.0, 1.0, samples=10)

# create a list with path points
path = []
n_path = 10
for i in range(n_path):
   yi = 0.7 + i * 0.6 / 10
   path.append(TrajectoryPt([dx, yi, angle]))
for i in range(10):
    xi = 1.0 - i * 0.3 / 10
    dxi = TolerancedNumber(xi, xi-0.1, xi+0.1, samples=3)
    yi = 1.3 + i * 0.3 / 10
    path.append(TrajectoryPt([dxi, yi, angle]))
fig1, ax1 = plt.subplots()
plot_path(ax1, path)

robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

path_js = cart_to_joint(robot1, path, check_collision = True, scene = sc1)

#%%

sub_path = []
for dof in range(3):
    Qi = [q[:, dof] for q in path_js]
    fi, pi = get_shortest_path(Qi)
    sub_path.append(pi)


fig, ax = plt.subplots(1, 3)
#plt.axis([0, 3, -3, 0])
#plt.axis([-0.8, 1, 0, 3])
for i, qs in enumerate(path_js):
    ind = np.linspace(0, 1, qs.shape[0])
    ind = np.ones(qs.shape[0]) * i
    q1 = qs[:, 0]
    q2 = qs[:, 1]
    q3 = qs[:, 2]
#    if i == 0:
#        ax.plot(ind, q2, 'b*')
#    if i == 19:
#        ax.plot(ind, q2, 'r.')
#    else:
    ax[0].plot(ind, q1, '*', color=(0.2, 0.2, 0.5, 0.2))
    ax[1].plot(ind, q2, '*', color=(0.2, 0.2, 0.5, 0.2))
    ax[2].plot(ind, q3, '*', color=(0.2, 0.2, 0.5, 0.2))
    
    ax[0].plot(i, sub_path[0][i], 'r*')
    ax[1].plot(i, sub_path[1][i], 'r*')
    ax[2].plot(i, sub_path[2][i], 'r*')

# find the best sequence of joint solutions in path_js
# currently total joint movement is minimized by default
#path_length, shortest_path_js = get_shortest_path(path_js)
#
#fig4, ax4 = plt.subplots()
#plt.title("The first solution")
#ax4.axis('equal')
#robot1.plot_path(ax4, shortest_path_js)
#plot_path(ax4, path, show_tolerance=False)
#plot_scene(ax4, sc1, 'r')