#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 12:41:14 2017

@author: jeroen
"""

import matplotlib.pyplot as plt
import numpy as np

from ppr.graph import get_shortest_path, create_graph, create_cost_matrices
from ppr.path import TolerancedNumber, TrajectoryPt, cart_to_joint, cart_to_joint_d
from ppr.geometry import Rectangle
from example_robots import Robot_3R, Robot_2P3R
from ppr.scene import plot_scene
# create trajectory
dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)

traj = []
N_traj = 10
for i in range(N_traj):
    yi = 0.7 + i * 0.6 / 10
    traj.append(TrajectoryPt([dx, yi, angle]))

""" 3 dof robot """
# test robot
#robot1 = Robot_3R([1, 1, 0.5], [0.02, 0.02, 0.02])
#
## plot nominal trajectory
#fig = plt.figure()
#ax = fig.gca()
#plt.axis('equal')
#plt.axis([-1, 3, -1, 3])
#xt = [tp.p_nominal[0] for tp in traj]
#yt = [tp.p_nominal[1] for tp in traj]
#ax.plot(xt, yt, '*')
#robot1.plot(ax, [0.1, 0.1, 0.1], 'k')
#
## create collision objects
#sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
#      Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
##sc1.append(Rectangle(1.09, 0.5, 0.2, 1.0, 0.0))
#sc2 = [Rectangle(0.5, 1.1, 0.1, 0.2, -0.3),
#      Rectangle(0.2, 1.0, 0.1, 0.5, 0.2)]
#plot_scene(ax, sc1, 'g')
#
#jt = cart_to_joint(robot1, traj, check_collision = True, scene = sc1)
#
#f, p = get_shortest_path(jt)
#print(f, p)
#q_path = [jt[i][p[i]] for i in range(len(jt))]
#
#robot1.plot_path(ax, q_path)

""" 5 dof robot """
traj = []
N_traj = 10
for i in range(N_traj):
    yi = 0.7 + i * 0.6 / 10 + 0.5
    traj.append(TrajectoryPt([0.9, yi, angle]))
# test robot
robot2 = Robot_2P3R([0.5, 0.5, 1, 1, 0.5])

# plot nominal trajectory
fig = plt.figure()
ax2 = fig.gca()
plt.axis('equal')
plt.axis([-1, 3, -1, 3])
xt = [tp.p_nominal[0] for tp in traj]
yt = [tp.p_nominal[1] for tp in traj]
ax2.plot(xt, yt, '*')
robot2.plot(ax2, [0.0, 0.0, 0.1, 0.1, 0.1], 'k')

# create collision objects
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
      Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
#sc1.append(Rectangle(1.09, 0.5, 0.2, 1.0, 0.0))
sc2 = [Rectangle(0.5, 1.1, 0.1, 0.2, -0.3),
      Rectangle(0.2, 1.0, 0.1, 0.5, 0.2)]
plot_scene(ax2, sc1, 'g')

jt = cart_to_joint_d(robot2, traj, check_collision = True, scene = sc1)
n_nodes = [el.shape[0] for el in jt]
print("Number of nodes: " + str(np.sum(n_nodes)))
n_edges = [n_nodes[i] * n_nodes[i+1] for i in range(len(n_nodes)-1)]
print("Number of edges: " + str(np.sum(n_edges)))
print(jt[0].shape)

#f, p = get_shortest_path(jt)
#print(f, p)
#q_path = [jt[i][p[i]] for i in range(len(jt))]
#
#robot2.plot_path(ax2, q_path)