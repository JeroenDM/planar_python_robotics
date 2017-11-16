#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 12:41:14 2017

@author: jeroen
"""

import matplotlib.pyplot as plt

from graph import get_shortest_path
from path import tolValue, TrajectoryPt, cart_to_joint
from robot import Robot
from geometry import Rectangle
from scene import plot_scene
# create trajectory
dx    = tolValue(1, 0.9, 1.1, samples=3)
angle = tolValue(0.0, -0.5, 0.5, samples=5)

traj = []
N_traj = 10
for i in range(N_traj):
    yi = 0.7 + i * 0.6 / 10
    traj.append(TrajectoryPt([dx, yi, angle]))

# test robot
robot1 = Robot([1, 1, 0.5], [0.02, 0.02, 0.02])

# plot nominal trajectory
fig = plt.figure()
ax = fig.gca()
plt.axis('equal')
plt.axis([-1, 3, -1, 3])
xt = [tp.p_nominal[0] for tp in traj]
yt = [tp.p_nominal[1] for tp in traj]
ax.plot(xt, yt, '*')
robot1.plot(ax, [0.1, 0.1, 0.1], 'k')

# create collision objects
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
      Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
#sc1.append(Rectangle(1.09, 0.5, 0.2, 1.0, 0.0))
sc2 = [Rectangle(0.5, 1.1, 0.1, 0.2, -0.3),
      Rectangle(0.2, 1.0, 0.1, 0.5, 0.2)]
plot_scene(ax, sc1, 'g')

jt = cart_to_joint(robot1, traj, check_collision = True, scene = sc1)

f, p = get_shortest_path(jt)
print(f, p)
q_path = [jt[i][p[i]] for i in range(len(jt))]

robot1.plot_path(ax, q_path)