#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 09:22:55 2018

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import Robot_3R
from ppr.path import TrajectoryPt, TolerancedNumber
from ppr.geometry import Rectangle

# ROBOT
robot1 = Robot_3R([2, 2, 2])

# PATH
dx    = np.linspace(3, 4, 10)
dy    = TolerancedNumber(1.0, 1.0, 1.1, samples=3)
angle = TolerancedNumber(0.0, -np.pi/2, np.pi/2, samples=3)
path1 = [TrajectoryPt([xi, dy, angle]) for xi in dx]

# COLLISION SCENE
sc1 = [Rectangle(3, 1.3, 2, 1, -0.1),
       Rectangle(3, 0.5, 2, 0.3, 0)]

from ppr.sampling import cart_to_joint, SolutionPoint

sol_pts = [SolutionPoint(tp) for tp in path1]
path_js = [sp.get_joint_solutions(robot1, check_collision=True, scene=sc1) for sp in sol_pts]

print([len(qp)for qp in path_js])

sol = get_shortest_path(path_js)
print(sol['success'])

fig2, ax2 = plt.subplots()
ax2.axis('equal')
robot1.plot_path(ax2, sol['path'])
for r in sc1: r.plot(ax2, 'g')
for tp in path1: tp.plot(ax2)
plt.show()

#%%
from ppr.path import TrajectoryPt, TolerancedNumber

a = TolerancedNumber(2, 1, 3, samples=10)
print(a.range)

a.add_samples(5)
print(a.range)