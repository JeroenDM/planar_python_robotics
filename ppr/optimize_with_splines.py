#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 15:09:51 2017

@author: jeroen
"""

import numpy as np
#import matplotlib.pyplot as plt
from scipy.optimize import fmin_slsqp, curve_fit
from scipy.interpolate import CubicSpline, LSQUnivariateSpline

from optimize import joint_motion_obj, path_ieq_con, reshape_path_vector

#=============================================================================
# Main function
#=============================================================================

def fit_initial_q(q):
    t = np.linspace(0, 1, 7)
    t = t[1:-1]
    xq = np.linspace(0, 1, len(q))
    return LSQUnivariateSpline(xq, q, t)

def get_optimal_trajectory(robot, path, q_path_init,
                           w={'joint_motion': 1.0,
                               'path_error'  : 0.0,
                               'torque'      : 0.0}):
    n_path = len(path)
    q_init = np.array(q_path_init).flatten()
    
    # setup the correct objective
    # I think this is faster than derictly using the coeffs
    # which would result in useless function evalutation if w = 0
    wb = [bool(w[key]) for key in w]
    if wb == [1, 0, 0]:
        def obj(x):
            n_path, qp = reshape_path_vector(x, n_dof=robot.ndof)
            return w['joint_motion'] * cs_joint_motion_obj(qp)
    else:
        raise ValueError("This type of objective is not implemented yet.")
    
    # setup inequality constraints
    def ieq_con(x):
        n_path, qp = reshape_path_vector(x, n_dof=robot.ndof)
        return cs_path_ieq_con(qp, robot, path)

    # Solve problem
    sol = fmin_slsqp(obj, 
                     q_init,
                     f_ieqcons=ieq_con)
    
    n_path, qp_sol = reshape_path_vector(sol, n_dof=robot.ndof)
    return qp_sol

#=============================================================================
# Objectives
    # wrap objectives to work with sampled splines
#=============================================================================

def cs_joint_motion_obj(qp, samples=20):
    n_v, n_dof = qp.shape
    t  = np.linspace(0, 1, n_v)
    ts = np.linspace(0, 1, samples)
    qps = sample_joint_path(qp, t, ts)
    return joint_motion_obj(qps)


#=============================================================================
# Constraints
    # wrap Constraints to work with sampled splines
#=============================================================================

def cs_path_ieq_con(qp, robot, path):
    n_v, n_dof = qp.shape
    t  = np.linspace(0, 1, n_v)
    ts = np.linspace(0, 1, len(path))
    qps = sample_joint_path(qp, t, ts)
    return path_ieq_con(qps, robot, path)


#=============================================================================
# Utilities
#=============================================================================

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



if __name__ == '__main__':
    import matplotlib.pyplot as plt
    from path import plot_path
    from scene import plot_scene
    from default_test_data import default_test_data
    
    r1, path, s1 = default_test_data()
    q_rand = np.random.rand(10, 3)
    
    print('Test main function')
    print('---------------------------------')
    qp_init = np.loadtxt('../data/initial_path.txt')
    
    tp = np.linspace(0, 1, len(path))
    t_small = np.linspace(0, 1, 5)
    qp_small = sample_joint_path(qp_init, tp, t_small)
    
    qp_sol = get_optimal_trajectory(r1, path, qp_small)
    qp2 = sample_joint_path(qp_small, t_small, tp)
    
    fig1, ax1 = plt.subplots(1, 2)
    fig1.suptitle('Initial vs optimized path')
    
    plot_scene(ax1[0], s1)
    plot_scene(ax1[1], s1)
    
    plot_path(ax1[0], path)
    plot_path(ax1[1], path)
    
    r1.plot_path(ax1[0], qp_init)
    r1.plot_path(ax1[1], qp2)
    for i in range(len(t_small)):
        r1.plot(ax1[1], qp_sol[i], 'r')
    
    fig2, ax2 = plt.subplots()
    for i in range(3):
        ax2.plot(tp, qp_init[:, i], t_small, qp_sol[:, i], 'o')
    
    print('Test objective')
    print('---------------------------------')
    o1 = cs_joint_motion_obj(qp_init)
    o2 = cs_joint_motion_obj(qp_init, samples=50)
    o3 = cs_joint_motion_obj(qp_small)
    print(o1, o2, o3)
    
    print('Test constraints')
    print('---------------------------------')
    c1 = cs_path_ieq_con(qp_init, r1, path)
    c2 = cs_path_ieq_con(qp_small, r1, path)
    print(c1.shape, c2.shape)
    print(c1[:5])
    print(c2[:5])
    
#    # define a time vector, assume constant sample time and total time of 10 s
#    t = np.linspace(0, 10, len(qp_init))
#    ts = np.linspace(0, 10, 100)
#    
#    T = get_joint_torque(r1, qp_init, t, ts)
#    
#    def obj(var):
#        n_path = int(len(var) / 3)
#        qp = var.reshape(n_path, 3)
#        T = get_joint_torque(r1, qp, t, ts)
#        return np.sum(T**2)
#    
#    def ieqcons(var):
#        n_path = int(len(var) / 3)
#        qp = var.reshape(n_path, 3)
#        return path_constraints(r1, qp, path, t, t)
    
#    sol = fmin_slsqp(obj, qp_init, f_ieqcons=ieqcons)

#%%

#n_path = len(path) + 2
#sol = sol.reshape(n_path, 3)
#q_sol = sample_joint_path(sol, t, ts)
#plt.plot(q_sol)
#
#
#fig5, ax5 = plt.subplots()
#plt.title("Optimized solution")
#ax5.axis('equal')
#r1.plot_path(ax5, sol.reshape(n_path, 3))
#plot_path(ax5, path, show_tolerance=False)
#plt.figure()
#plt.title("Initial joint motion")
#plt.plot(t, qpi[:, 0], '.C0')
#plt.plot(t, qpi[:, 1], '.C1')
#plt.plot(t, qpi[:, 2], '.C2')
#te = np.linspace(0, 10, 100)
#plt.plot(te, qpi_spline[0](te), 'C0')
#plt.plot(te, qpi_spline[1](te), 'C1')
#plt.plot(te, qpi_spline[2](te), 'C2')

#Ts = get_joint_torque(r1, sol, t, ts)
#plt.figure()
#plt.plot(Ts)
#
#plt.figure()
#plt.plot(q_sol)
#
#qi, dqi, ddqi = sample_trajectory(qp_init, t, ts)
#q, dq, ddq = sample_trajectory(sol, t, ts)
#plt.figure()
#plt.plot(q)
#plt.figure()
#plt.plot(dq)
#plt.figure()
#plt.plot(dqi)
#plt.figure()
#plt.title("Solution")
#plt.plot(t, qp[:, 0], '.C0')
#plt.plot(t, qp[:, 1], '.C1')
#plt.plot(t, qp[:, 2], '.C2')
#te = np.linspace(0, 10, 100)
#plt.plot(te, qp_spline[0](te), 'C0')
#plt.plot(te, qp_spline[1](te), 'C1')
#plt.plot(te, qp_spline[2](te), 'C2')