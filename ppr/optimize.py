#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 21 11:59:33 2017

@author: jeroen
"""


import numpy as np
from scipy.optimize import minimize, fmin_slsqp

from scipy.interpolate import CubicSpline

def get_optimal_trajectory(robot, path, path_js_init, c_torque=1.0, c_position=1.0):
    n_path = len(path)
    q_init = np.array(path_js_init).flatten()
    
    def obj(x):
        return c_torque * torque_cost(robot, x) + c_position * path_pos_error(robot, x, path)

    sol = fmin_slsqp(obj, 
                     q_init,
                     f_ieqcons=lambda x: path_constraints(robot, x, path))
    
    qsol = sol.reshape(n_path, 3)
    dqs, ddqs = q_derivatives(qsol)
    return qsol, dqs, ddqs

def optimal_spline_wrap(robot, path, path_js_init):
    qp = path_js_init
    n_path = len(qp)
    t_end = (n_path - 1) * 0.1
    tp = np.linspace(0, t_end, n_path)
    q_splines = []
    for i in range(3):
        q_splines.append(CubicSpline(tp, qp[:, i]))
    
    return 0
    

def rk4(ode,h,x,u):
  k1 = ode(x,       u)
  k2 = ode(x+h/2*k1,u)
  k3 = ode(x+h/2*k2,u)
  k4 = ode(x+h*k3,  u)
  return x + h/6 * (k1 + 2*k2 + 2*k3 + k4)

def path_pos_error(robot, q_path, path):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
    con = []
    for i, tp in enumerate(path):
        pp = tp.p_nominal
        pfk = robot.fk(qp[i])
        con.append(np.sum((pp - pfk)**2))
    con = np.array(con).flatten()
    return np.sum(con**2)

def path_cost(q_path):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
    dqp = np.diff(qp, axis=0)
    
    return np.sum(np.abs(dqp))



def obj(robot, q_path, x_path):
    return path_pos_error(robot, q_path, x_path) + 0.1 * path_cost(q_path)

def path_constraints(robot, q_path, path, tol=1e-6):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
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

def path_c2(robot, q_path, path):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
    con = []
    for i, tp in enumerate(path):
        pp = tp.p_nominal
        pfk = robot.fk(qp[i])
        con.append(np.sum((pp - pfk)**2))
    return np.array(con).flatten()

def torque_cost(robot, q_path):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
    dqp, ddqp = q_derivatives(qp)
    tau = []
    for i in range(n_path):
        tau.append(robot.euler_newton(qp[i], dqp[i], ddqp[i]))
    tau = np.array(tau)
    return np.sum(tau**2)

def jerk_cost(robot, q_path):
    n_path = int(len(q_path) / 3)
    qp = q_path.reshape(n_path, 3)
    dqp, ddqp = q_derivatives(qp)
    tau = []
    for i in range(n_path):
        tau.append(robot.euler_newton(qp[i], dqp[i], ddqp[i]))
    tau = np.array(tau)
    jerk = np.gradient(tau, axis=0)
    return np.sum(jerk**2)

def q_derivatives(q, dt=0.1):
    dq = []
    ddq = []
    for i in range(3):
        dq.append( np.gradient(q[:, i]) * dt)
        ddq.append(np.gradient(dq[i])   * dt)
    dq = np.array(dq).T
    ddq = np.array(ddq).T
    return dq, ddq

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from robot import Robot
    from path import TolerancedNumber, TrajectoryPt, plot_path
    
    print("-----test optimize.py-----")
    print("--------------")
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -2, 3])
    
    qr = np.linspace(0, 1, 10)
    qt = np.array([qr, qr, qr]).T
    
    r1 = Robot(['r', 'r', 'r'], [1, 1, 0.5], [0, 0, 0])
    
    
    # use minimize to solve ik ?    
    qt = np.array([[0.3, 0.2, -0.1], [0.4, 0.2, -0.1]])
    xt = np.array([r1.fk([0.3, 0.2, -0.1]), r1.fk([0.4, 0.2, -0.1])])
    print(xt)
    
    print(path_pos_error(r1, qt.flatten(), xt))
    print(path_cost(qt.flatten()))
    print(obj(r1, qt.flatten(), xt))
    
#    sol = minimize(lambda x: path_pos_error(r1, x, xt), [0, 0, 0, 0, 0, 0])
#    print(sol)
    
    # create path
    N = 10
    path = []
    for i in range(N):
        path.append([1.5, i/10, 1])
    
#    q_init = np.zeros((N, 3)).flatten()
    q_init = np.zeros((N, 3))
    q_init[:, 0] = 0.5
    q_init = q_init.flatten()
    
#    path = np.array(path)
##    sol = minimize(lambda x: path_pos_error(r1, x, path), q_init)
#    
#    sol = minimize(lambda x: obj(r1, x, path), q_init)
#    print(sol['x'])
#    q_sol = sol['x'].reshape(N, 3)
#    
#    r1.plot_path_kinematics(ax, q_sol)
    
    # create tolerances for x-position and orientation
    dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
    angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)
    
    # create a list with path points
    path = []
    n_path = 10
    for i in range(n_path):
       yi = 0.7 + i * 0.6 / 10
       path.append(TrajectoryPt([dx, yi, angle]))
    
    c = path_constraints(r1, q_init, path)
    print(c)
    
    plot_path(ax, path, show_tolerance=False)
    
    def feq(q):
        return path_c2(r1, q, path)
    
    sol1 = fmin_slsqp(path_cost, q_init, f_eqcons=feq)
#    r1.plot_path_kinematics(ax, sol1.reshape(10, 3))
    print(sol1)
    qsol1 = sol1.reshape(10, 3)
    
    r1.set_link_inertia([1, 1, 1], [0.5, 0.5, 0.25], [0.05, 0.05, 0.05])
    print(torque_cost(r1, q_init))
    
#    def fieq(q):
#        return path_constraints(r1, q, path)
#    
#    def obj2(q):
#        return torque_cost(r1, q)
#    sol2 = fmin_slsqp(obj2, sol1, f_ieqcons=fieq)
    qsol2, dq, ddq = get_optimal_trajectory(r1, path, qsol1)
    r1.plot_path_kinematics(ax, qsol2)
    print(qsol2)
    
#    dq, ddq = q_derivatives(qsol2)
#    dq = []
#    ddq = []
#    for i in range(3):
#        dq.append(np.gradient(qsol2[:, i]))
#        ddq.append(np.gradient(dq[i]))
#    dq = np.array(dq).T
#    ddq = np.array(ddq).T
    
    fig1, ax1 = plt.subplots(1, 3)
    ax1[0].plot(qsol2)
    ax1[1].plot(dq)
    ax1[2].plot(ddq)
    
    
    tau = []
    for i in range(n_path):
        tau.append(r1.euler_newton(qsol2[i], dq[i], ddq[i]))
    tau = np.array(tau)
    plt.figure()
    plt.title('Torque')
    plt.plot(tau)
    
#def path_pos_error(robot, q_path, x_path):
#    n_path = int(len(q_path) / 3)
#    xp = np.array([robot.fk(q) for q in q_path.reshape(n_path, 3)])
#    xp = xp[:, :2].flatten()
#    x_path = x_path[:, :2].flatten()
#    
#    return np.sum((xp - x_path)**2)   
# from scipy.interpolate import CubicSpline, interp1d  
#from scipy.signal import resample
#def derivatives(t, x):
#    """ Calculate derivative along second axis,
#    or first axis if one dimensional input
#    """
#    
#    x = np.array(x)
#    sh = x.shape
#    if len(sh) == 1:
#        N = sh[0]
#        M = 1
#    else:
#        M, N = sh
#    
#    Ns = 30
#    ts = np.linspace(0, t[-1], Ns)
#    f = interp1d(t, x)
#    xs = f(ts)
#    print(xs.shape)
#    sp = []
#    for i in range(M):
#        cs = CubicSpline(ts, xs[i], bc_type='clamped')
#        sp.append(cs)
#    return ts, xs, sp 
#    #x = np.arange(10)
#    x = np.linspace(0, 10, 10)
#    y = np.sin(x)
#    cs = CubicSpline(x, y)
#    xs = np.arange(-0.5, 9.6, 0.1)
#    plt.figure(figsize=(6.5, 4))
#    plt.plot(x, y, 'o', label='data')
#    plt.plot(xs, np.sin(xs), label='true')
#    plt.plot(xs, cs(xs), label="S")
#    plt.plot(xs, cs(xs, 1), label="S'")
#    plt.plot(xs, cs(xs, 2), label="S''")
#    plt.plot(xs, cs(xs, 3), label="S'''")
#    plt.xlim(-0.5, 9.5)
#    plt.legend(loc='lower left', ncol=2)
#    plt.show()
#    
#    t = np.linspace(0, 10, 10)
#    q = np.array([y, y, -y])
#    ts, xs, s = derivatives(t, q)
#    
#    fig5, ax5 = plt.subplots()
#    ax5.plot(ts, xs[0], '.')
#    qs = s[0](ts)
#    dq = s[0](ts, 1)
#    ddq = s[0](ts, 2)
#    ax5.plot(ts, qs, ts, dq, ts, ddq)