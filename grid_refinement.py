#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Apr  5 10:04:41 2018

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import Robot_2P3R,Robot_3R
from ppr.path import TrajectoryPt, TolerancedNumber
from ppr.geometry import Rectangle
from ppr.path import cart_to_joint
from ppr.path import get_shortest_path

def get_new_bounds(l, u, m, red=4):
    """ create new interval smaller than the old one (l, u)
    reduced in length by a factor red.
    m is the value around wich the new interval should be centered
    the new interval may not go outside the old bounds
    """
    delta = abs(u - l) / red
    l_new = max(m - delta, l)
    u_new = min(m + delta, u)
    return l_new, u_new

def resample_trajectory_point(tp, pfk, *arg, **kwarg):
    """ create a new trajectory point with smaller bounds,
    but same sample number
    use the value from the forward kinematics pfk as the center
    of the new interval
    """
    p_new = []
    for i, val in enumerate(tp.p):
        if tp.hasTolerance[i]:
            # check for rounding errors on pfk
            if pfk[i] < val.l:
                pfk[i] = val.l
            if pfk[i] > val.u:
                pfk[i] = val.u
            l, u = get_new_bounds(val.l, val.u, pfk[i], *arg, **kwarg)
            val_new = TolerancedNumber(pfk[i], l, u, samples=val.s)
        else:
            val_new = val
        p_new.append(val_new)
    return TrajectoryPt(p_new)

def resample_path(path, q_sol, robot, *arg, **kwarg):
    poses = [robot.fk(q) for q in q_sol]
    path_new = []
    for i, tp in enumerate(path1):
        path_new.append(resample_trajectory_point(tp, poses[i], *arg, **kwarg))
    
    return path_new

# ROBOT
robot1 = Robot_3R([2, 2, 2])

# PATH
dx    = np.linspace(3, 5, 10)
dy    = TolerancedNumber(1.0, 0.8, 1.2, samples=5)
angle = TolerancedNumber(0.0, -np.pi/2, np.pi/2, samples=25)
path1 = [TrajectoryPt([xi, dy, angle]) for xi in dx]

# COLLISION SCENE
sc1 = [Rectangle(3, 1.3, 2, 1, -0.1),
       Rectangle(3, 0.5, 2, 0.3, 0)]

#%% initial solution
Q1 = cart_to_joint(robot1, path1, check_collision=True, scene=sc1)
print([len(qp) for qp in Q1])

sol_init = get_shortest_path(Q1)

fig2, ax2 = plt.subplots()
ax2.axis('equal')
#robot1.plot_path_kinematics(ax2, path_js[2])
robot1.plot_path(ax2, sol_init['path'])
for r in sc1: r.plot(ax2, 'k')
for tp in path1: tp.plot(ax2)

plt.title('Initial solution to path following task')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

#%% initial solution, different scene
sc2 = [Rectangle(3, 1.3, 2, 1, -0.1),
       Rectangle(3, 0.5, 2, 0.3, 0),
       Rectangle(1, -1, 1, 1.5, 0)]

Q1 = cart_to_joint(robot1, path1, check_collision=True, scene=sc2)
print([len(qp) for qp in Q1])

sol_init = get_shortest_path(Q1)

fig2, ax2 = plt.subplots()
ax2.axis('equal')
#robot1.plot_path_kinematics(ax2, path_js[2])
robot1.plot_path(ax2, sol_init['path'])
for r in sc2: r.plot(ax2, 'k')
for tp in path1: tp.plot(ax2)

plt.title('Initial solution to path following task')
plt.xlabel('X')
plt.ylabel('Y')
plt.show()

#%% iterative imporvement
current_path = path1
costs = []
paths = []
for i in range(6):
    Q = cart_to_joint(robot1, current_path, check_collision=True, scene=sc1)
    sol = get_shortest_path(Q)
    if sol['success']:
        current_path = resample_path(current_path, sol['path'], robot1, red=10)
        costs.append(sol['length'])
        paths.append(current_path)
    else:
        print("No solution in graph found")

#%%
plt.plot(costs, 'ko-')
plt.xlabel('Iteration [-]', fontsize=18)
plt.ylabel('Path cost [rad]', fontsize=18)
#plt.ylim([0, 2.2])
plt.show()

#%%

fk_sol = []
for q in sol_init['path']:
    fk_sol.append(robot1.fk(q))
fk_sol = np.vstack(fk_sol)

figs, axs = plt.subplots(2, 1)
for tp in path1: tp.plot(axs[0])
axs[0].plot(fk_sol[:, 0], fk_sol[:, 1], 'ro')
axs[0].set_title('Original path tolerance')
#axs[0].set_xlabel('X')
axs[0].set_ylabel('Y')

for tp in paths[1]: tp.plot(axs[1])
axs[1].set_title('Adjusted path tolerance last iteration')
axs[1].set_xlabel('X')
axs[1].set_ylabel('Y')

# more space for title second plot
plt.subplots_adjust(hspace=0.5)
plt.show()

#%% plot joint angles

t = np.linspace(0, 1, len(sol['path'])-1)
jv0 = np.vstack(sol_init['path'])
jv = np.vstack( sol['path'] )

c0 = np.sum(np.abs(np.diff(jv0, axis=0)), axis=1)
ct = np.sum(np.abs(np.diff(jv, axis=0)), axis=1)

#fig, axs = plt.subplots(3,1, figsize=(15, 15), facecolor='w', edgecolor='k')
#axs[0].plot(jv[:, 0])

fig, ax = plt.subplots()
#ax.set_ylim([-np.pi, np.pi])
#ax.plot(t, np.diff(jv[:, 0]), 'k',
#        t, np.diff(jv[:, 1]), 'k--',
#        t, np.diff(jv[:, 2]), 'k-.')
#ax.legend(['1', '2', '3'])
#
#ax.plot(t, np.diff(jv0[:, 0]), 'b',
#        t, np.diff(jv0[:, 1]), 'b--',
#        t, np.diff(jv0[:, 2]), 'b-.')

ax.plot(t, c0, 'b', t, c, 'k')
plt.show()


#%% optimisation approach

from ppr.optim import get_optimal_trajectory

q_init = sol_init['path']
q_opt, dq_opt, ddq_opt = get_optimal_trajectory(robot1, path1, q_init, check_collision=True, scene=sc1)


#%%
fig3, ax3 = plt.subplots()
ax3.axis('equal')
robot1.plot_path_kinematics(ax3, q_opt)
for r in sc1: r.plot(ax3, 'g')
for tp in path1: tp.plot(ax3)
plt.show()

