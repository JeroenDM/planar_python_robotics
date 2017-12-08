#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 15:09:51 2017

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin_slsqp
from scipy.interpolate import CubicSpline

""" setup robot and initial path """

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

from example_robots import Robot_3R
robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])
robot1.set_link_inertia([1, 1, 1], [0.5, 0.5, 0.25], [0.05, 0.05, 0.05])

qpi = np.loadtxt('initial_path.txt')

#fig, ax = plt.subplots()
#robot1.plot_path(ax, q_path_init)

""" optimization functions """
def points_to_spline(points, x):
    """ convert a 1D numpy array to a spline with these values as coefs  """
    sp = CubicSpline(x, points, bc_type='clamped')
    return sp

def path_to_splines(qp, t):
    return [points_to_spline(qp[:, i], t) for i in range(3)]

def sample_joint_path(qp, t, t_sample):
    qp_spline = path_to_splines(qp, t)
    qp =   np.array( [qp_spline[i](t_sample)     for i in range(3)] ).T
    return qp

def sample_trajectory(qp, t, t_sample):
    qp_spline = path_to_splines(qp, t)
    qp =   np.array( [qp_spline[i](t_sample)     for i in range(3)] ).T
    dqp =  np.array( [qp_spline[i](t_sample, 1)  for i in range(3)] ).T
    ddqp = np.array( [qp_spline[i](t_sample, 2)  for i in range(3)] ).T
    
    return qp, dqp, ddqp

def get_joint_torque(robot, qp, t, t_sample):
    q, dq, ddq = sample_trajectory(qp, t, t_sample)
    
    n_sample = len(t_sample)
    torque = np.zeros((n_sample, robot.ndof))
    for i in range(n_sample):
        torque[i] = robot.euler_newton(q[i], dq[i], ddq[i])
    
    return torque

def path_constraints(robot, qp, path, t, t_sample, tol=1e-6):
    qp = sample_joint_path(qp, t, t_sample)
    con = []
    for i, tp in enumerate(path):
        hast = tp.hasTolerance
        pfk = robot.fk(qp[i])
        for i in range(tp.dim):
            if hast[i]:
                con.append(-pfk[i] + tp.p[i].u)
                con.append( pfk[i] - tp.p[i].l)
            else:
                con.append(-pfk[i] + tp.p[i] + tol)
                con.append( pfk[i] - tp.p[i] - tol)
    return np.array(con)

""" setup and run problem """

# define a time vector, assume constant sample time and total time of 10 s
t = np.linspace(0, 10, len(qpi))
ts = np.linspace(0, 10, 100)

T = get_joint_torque(robot1, qpi, t, ts)

def obj(var):
    n_path = int(len(var) / 3)
    qp = var.reshape(n_path, 3)
    T = get_joint_torque(robot1, qp, t, ts)
    return np.sum(T**2)

def ieqcons(var):
    n_path = int(len(var) / 3)
    qp = var.reshape(n_path, 3)
    return path_constraints(robot1, qp, path, t, t)

sol = fmin_slsqp(obj, qpi, f_ieqcons=ieqcons)

#%%

sol = sol.reshape(n_path, 3)
q_sol = sample_joint_path(sol, t, ts)
plt.plot(q_sol)


fig5, ax5 = plt.subplots()
plt.title("Optimized solution")
ax5.axis('equal')
robot1.plot_path(ax5, sol.reshape(n_path, 3))
plot_path(ax5, path, show_tolerance=False)
#plt.figure()
#plt.title("Initial joint motion")
#plt.plot(t, qpi[:, 0], '.C0')
#plt.plot(t, qpi[:, 1], '.C1')
#plt.plot(t, qpi[:, 2], '.C2')
#te = np.linspace(0, 10, 100)
#plt.plot(te, qpi_spline[0](te), 'C0')
#plt.plot(te, qpi_spline[1](te), 'C1')
#plt.plot(te, qpi_spline[2](te), 'C2')

Ts = get_joint_torque(robot1, sol, t, ts)
plt.figure()
plt.plot(Ts)

plt.figure()
plt.plot(q_sol)

qi, dqi, ddqi = sample_trajectory(qpi, t, ts)
q, dq, ddq = sample_trajectory(sol, t, ts)
plt.figure()
plt.plot(q)
plt.figure()
plt.plot(dq)
plt.figure()
plt.plot(dqi)
#plt.figure()
#plt.title("Solution")
#plt.plot(t, qp[:, 0], '.C0')
#plt.plot(t, qp[:, 1], '.C1')
#plt.plot(t, qp[:, 2], '.C2')
#te = np.linspace(0, 10, 100)
#plt.plot(te, qp_spline[0](te), 'C0')
#plt.plot(te, qp_spline[1](te), 'C1')
#plt.plot(te, qp_spline[2](te), 'C2')