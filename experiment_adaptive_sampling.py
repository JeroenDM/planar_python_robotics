#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 09:22:55 2018

@author: jeroen
"""

#import numpy as np
#import matplotlib.pyplot as plt
#from ppr.robot import Robot_3R
#from ppr.path import TrajectoryPt, TolerancedNumber
#from ppr.geometry import Rectangle
#from ppr.sampling import SolutionPoint, get_shortest_path, cart_to_joint_dynamic
#
## ROBOT
#robot1 = Robot_3R([2, 2, 2])
#
## PATH
#dx    = np.linspace(3, 4, 10)
#dy    = TolerancedNumber(1.0, 1.0, 1.1, samples=3)
#angle = TolerancedNumber(0.0, -np.pi/2, np.pi/2, samples=3)
#path1 = [TrajectoryPt([xi, dy, angle]) for xi in dx]
#
## COLLISION SCENE
#sc1 = [Rectangle(3, 1.3, 2, 1, -0.1),
#       Rectangle(3, 0.5, 2, 0.3, 0)]
#
#
## GRAPH CREATION
##sol_pts = [SolutionPoint(tp) for tp in path1]
##
##for sp  in sol_pts:
##    max_iters = 10
##    while (sp.num_js < 10 and max_iters > 0):
##        sp.add_joint_solutions(robot1, 10, check_collision=True, scene=sc1)
##        max_iters -= 1
##    print(max_iters)
##
##path_js = [sp.get_joint_solutions() for sp in sol_pts]
#
#path_js = cart_to_joint_dynamic(robot1, path1, check_collision = True, scene=sc1)
#print([len(qp) for qp in path_js])
#
#min_num_sol = 20
#incr_num_sol = 5        
#
#print([len(qp)for qp in path_js])
#
## FIND SOLUTION
#sol = get_shortest_path(path_js)
#print(sol['success'])
#
#fig2, ax2 = plt.subplots()
#ax2.axis('equal')
#robot1.plot_path(ax2, sol['path'])
#for r in sc1: r.plot(ax2, 'g')
#for tp in path1: tp.plot(ax2)
#plt.show()

#%%
import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import Robot, Robot_3R, Robot_2P3R
from ppr.path import TrajectoryPt, TolerancedNumber
from ppr.geometry import Rectangle
from ppr.sampling import cart_to_joint, SolutionPoint
from ppr.sampling import get_shortest_path, cart_to_joint_dynamic

np.random.seed(41)

# ROBOT
robot1 = Robot_2P3R([4, 0.9, 2, 1.2, 1])
robot1.set_joint_limits([(2.0, 3.0), (0.2, 0.9)])
robot1.ik_samples = [3, 5]

# PATH
dx    = TolerancedNumber(0.5, 0.3, 0.8, samples=4)
dy    = np.linspace(2, 2.5, 5)
angle = TolerancedNumber(0.0, -np.pi, np.pi, samples=20)
path1 = [TrajectoryPt([dx, yi, angle]) for yi in dy]

# COLLISION SCENE
sc1 = [Rectangle(1, 1, 1, 1.5, 0),
       Rectangle(3, 1, 1, 2.2, 0),
       Rectangle(0, 3.2, 4, 0.5, 0),
       Rectangle(0, 1, 0.2, 3.2, 0),
       Rectangle(0.2, 1, 0.8, 0.5, 0)]


##path_js = cart_to_joint(robot1, path1, check_collision=True, scene=sc1)
# GRAPH CREATION
#sol_pts = [SolutionPoint(tp) for tp in path1]
## inital joint limits
#for sp in sol_pts:
#    sp.jl = robot1.jl
#
#for sp  in sol_pts:
#    max_iters = 50
#    while (sp.num_js < 100 and max_iters > 0):
#        sp.add_joint_solutions(robot1, 10, check_collision=True, scene=sc1)
#        max_iters -= 1
#    print(max_iters)
#
#path_js = [sp.get_joint_solutions() for sp in sol_pts]

path_js = cart_to_joint_dynamic(robot1, path1, check_collision = True, scene=sc1)
print([len(qp) for qp in path_js])

sol = get_shortest_path(path_js)
print(sol['success'])

fig2, ax2 = plt.subplots()
ax2.axis('equal')
#robot1.plot_path_kinematics(ax2, path_js[2])
robot1.plot_path(ax2, sol['path'])
for r in sc1: r.plot(ax2, 'g')
for tp in path1: tp.plot(ax2)
plt.show()

#%%

#robot1 = Robot_2P3R([4, 0.9, 2, 1.2, 1])
#robot1.set_joint_limits([(2.0, 3.0), (0.2, 0.9)])
#robot1.ik_samples = [3, 5]
#
#tp1 = TrajectoryPt([TolerancedNumber(0.5, 0.3, 0.8, samples=5),
#                    2,
#                    TolerancedNumber(0.0, -np.pi, np.pi, samples=10)])
#
#sp1 = SolutionPoint(tp1)
#sp1.jl = robot1.jl
#sp1.is_redundant = True
#
#sp1.add_joint_solutions(robot1, 4, 1)
#res = sp1.get_joint_solutions()
#print(res)
#print(res.shape)






