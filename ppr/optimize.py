#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Solve numerical optimization problem for robot path

@author: jeroen
"""


import numpy as np
from scipy.optimize import fmin_slsqp


#=============================================================================
# Main functions to run the problem
#=============================================================================
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
            return w['joint_motion'] * joint_motion_obj(qp)
    elif wb == [0, 0, 1]:
        def obj(x):
            n_path, qp = reshape_path_vector(x, n_dof=robot.ndof)
            return w['torque'] * torque_obj(qp, robot)
    else:
        raise ValueError("This type of objective is not implemented yet.")
    
    # setup inequality constraints
    def ieq_con(x):
        n_path, qp = reshape_path_vector(x, n_dof=robot.ndof)
        return path_ieq_con(qp, robot, path)

    # Solve problem
    sol = fmin_slsqp(obj, 
                     q_init,
                     f_ieqcons=ieq_con)
    
    n_path, qp_sol = reshape_path_vector(sol, n_dof=robot.ndof)
    dqs, ddqs = q_derivatives(qp_sol)
    return qp_sol, dqs, ddqs
    
#=============================================================================
# Objectives
# function structure : xxxx_obj(opt_var, params ...)
    # xxxxx   : name, the thing that is minimized
    # opt_var : the optimizationsv variables
    # parmas  : parameters, such as robot, path, coefficients, ....
# returns
    # a a scalar that has to be minimized
#=============================================================================

def path_error_obj(q_path, robot, path):
    con = []
    for i, tp in enumerate(path):
        pp = tp.p_nominal
        pfk = robot.fk(q_path[i])
        con.append(np.sum((pp - pfk)**2))
    con = np.array(con).flatten()
    return np.sum(con**2)

def joint_motion_obj(q_path):
    dqp = np.diff(q_path, axis=0)
    
    return np.sum(np.abs(dqp))

def torque_obj(q_path, robot):
    n_path, ndof = q_path.shape
    dqp, ddqp = q_derivatives(q_path)
    tau = np.zeros((n_path, robot.ndof))
    for i in range(n_path):
        tau[i, :] = robot.euler_newton(q_path[i], dqp[i], ddqp[i])
    return np.sum(tau**2)

#=============================================================================
# Constraints
    # function structure : xxxx_eq_con(opt_var, params ...)
    #                    : xxxx_ieq_con(opt, var, params ...)
    # xxxxx   : name, the thing that causes the constraints
    # opt_var : the optimizationsv variables
    # parmas  : parameters, such as robot, path, coefficients, ....
# returns
    # equality constraints: a numpy array, each element has to be zero
    # inequality constraints: a numpy array, each elment is positive when
    # the constraints is satisfied
#=============================================================================

def path_ieq_con(q_path, robot, path, tol=1e-6):
    con = []
    for i, tp in enumerate(path):
        has_tol = tp.hasTolerance
        pfk = robot.fk(q_path[i])
        for i in range(tp.dim):
            if has_tol[i]:
                con.append(-pfk[i] + tp.p[i].u)
                con.append( pfk[i] - tp.p[i].l)
            else:
                con.append(-pfk[i] + tp.p[i] + tol)
                con.append( pfk[i] - tp.p[i] - tol)
    return np.array(con)

#def path_error_eq_con(robot, q_path, path):
#    n_path = int(len(q_path) / 3)
#    qp = q_path.reshape(n_path, 3)
#    con = []
#    for i, tp in enumerate(path):
#        pp = tp.p_nominal
#        pfk = robot.fk(qp[i])
#        con.append(np.sum((pp - pfk)**2))
#    return np.array(con).flatten()



#def jerk_cost(robot, q_path):
#    n_path = int(len(q_path) / 3)
#    qp = q_path.reshape(n_path, 3)
#    dqp, ddqp = q_derivatives(qp)
#    tau = []
#    for i in range(n_path):
#        tau.append(robot.euler_newton(qp[i], dqp[i], ddqp[i]))
#    tau = np.array(tau)
#    jerk = np.gradient(tau, axis=0)
#    return np.sum(jerk**2)

#=============================================================================
# Utility functions
#=============================================================================
    
def reshape_path_vector(v_flat, n_dof=3):
    """ Convert flat vector to n_path x n_dof path array
    
    P is the number of discrete path points.
    This reshaping if often needed to formulate optimization problems.
    The standard optimization variable shape in scipy.optimize is a
    1D vector.
    
    Parameters
    ----------
    v_flat : ndarray
        Vector of length n_path*ndof containing the joint position vector for
        every path point [q1, q2, ...] with q1 of length ndof.
    ndof : int
        Degrees of freedom of robot.
    
    Returns
    -------
    ndarray
        Array of shape (n_path, ndof) every row containt the joint positions for
        a single path point.
    """
    n_path = int(len(v_flat) / n_dof)
    return n_path, v_flat.reshape(n_path, n_dof)

def q_derivatives(q, dt=0.1):
    """ Calculate joint speed and acceleration
    
    Based on a given path, caculate speed and acceleration using the
    numpy.gradient function. A constant sample time dt is assumed.
    
    Parameters
    ----------
    q : ndarray
        Array of shape (n_path, ndof) every row containt the joint positions for
        a single path point.
    dt : float
        Sample time for joint position path
    
    Returns
    -------
    tuple
        (dq, ddq) the joint speed and acceleration along the path.
        Arrays with the same shape as the input path array q.
    """
    dq = np.gradient(q, dt, axis=0)
    ddq = np.gradient(dq, dt, axis=0)
    return dq, ddq

#=============================================================================
# Testing
#=============================================================================
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from path import plot_path
    from scene import plot_scene
    from default_test_data import default_test_data
    
    r1, path, s1 = default_test_data()
    q_rand = np.random.rand(10, 3)
    
    print('Test main function')
    print('---------------------------------')
    qp_init = np.loadtxt('../data/initial_path.txt')
    
    # default case
    qp_sol, dqs, ddqs = get_optimal_trajectory(r1, path, qp_init)
    print(qp_sol)
    
    oi = joint_motion_obj(qp_init)
    os = joint_motion_obj(qp_sol)
    print('objective value improved? ' + str(oi) + ' to ' + str(os))
    
    fig1, ax1 = plt.subplots(1, 2)
    fig1.suptitle('Initial vs optimized path')
    
    plot_scene(ax1[0], s1)
    plot_scene(ax1[1], s1)
    
    plot_path(ax1[0], path)
    plot_path(ax1[1], path)
    
    r1.plot_path(ax1[0], qp_init)
    r1.plot_path(ax1[1], qp_sol)
    
    # torque objective
    weights = {'joint_motion': 0.0,
               'path_error'  : 0.0,
               'torque'      : 1.0}
    qp_sol, dqs, ddqs = get_optimal_trajectory(r1, path, qp_init, w=weights)
    print(qp_sol)
    
    oi = joint_motion_obj(qp_init)
    os = joint_motion_obj(qp_sol)
    print('objective value improved? ' + str(oi) + ' to ' + str(os))
    
    fig2, ax2 = plt.subplots(1, 2)
    fig2.suptitle('Minimal torque: Initial vs optimized path')
    
    plot_scene(ax2[0], s1)
    plot_scene(ax2[1], s1)
    
    plot_path(ax2[0], path)
    plot_path(ax2[1], path)
    
    r1.plot_path(ax2[0], qp_init)
    r1.plot_path(ax2[1], qp_sol)
    
    print('Test objectives')
    print('---------------------------------')
    o1 = path_error_obj(q_rand, r1, path)
    o2 = joint_motion_obj(q_rand)
    o3 = torque_obj(q_rand, r1)
    print(o1, o2, o3)
    
    print('Test constraints')
    print('---------------------------------')
    c1 = path_ieq_con(q_rand, r1, path)
    print(c1.shape)
    print(c1[:5])
    
    print('Test utils')
    print('---------------------------------')
    a_in = np.arange(1, 13)
    a_out = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9], [10, 11, 12]])
    n_path, a_test = reshape_path_vector(a_in)
    np.testing.assert_equal(n_path, 4)
    np.testing.assert_equal(a_test, a_out)
    
    t = np.linspace(0, 10, 50)
    dt = t[1]-t[0]
    x   = np.array([ np.sin(t),  np.cos(t), t**3     ]).T
    dx  = np.array([ np.cos(t), -np.sin(t), 3 * t**2 ]).T
    ddx = np.array([-np.sin(t), -np.cos(t), 6 * t    ]).T
    dx_test, ddx_test = q_derivatives(x, dt=dt)
    
    fig, ax = plt.subplots(3)
    fig.suptitle('Exact and approximate first derivative')
    for i in range(3):
        ax[i].plot(t, dx[:, i], t, dx_test[:, i], '.')

    fig2, ax2 = plt.subplots(3)
    fig2.suptitle('Exact and approximate second derivative')
    for i in range(3):
        ax2[i].plot(t, ddx[:, i], t, ddx_test[:, i], '.')
    
#    np.testing.assert_allclose(dx_test[10:40, :], dx[10:40, :], atol=0.01)
#    np.testing.assert_allclose(ddx_test[10:40, :], ddx[10:40, :], astol=0.01)
    
    
    