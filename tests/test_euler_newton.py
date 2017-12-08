#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 12:59:35 2017

@author: jeroen
"""

import matplotlib.pyplot as plt
import numpy as np
import numpy.testing as np_test

import os
os.chdir('../')
from ppr.robot import Robot

""" test input """
def q(t, T = 10.0, q_max=5.0):
    phase = -np.pi / 2
    omega = 2 * np.pi / T
    return q_max * (np.sin(omega * t + phase) + 1)

def dq(t, T=10.0, q_max=5.0):
    phase = -np.pi / 2
    omega = 2 * np.pi / T
    return q_max * omega * np.cos(omega * t + phase)

def ddq(t, T=10.0, q_max=5.0):
    phase = -np.pi / 2
    omega = 2 * np.pi / T
    return -q_max * omega**2 * np.sin(omega * t + phase)

t = np.linspace(0, 10)
plt.figure()
plt.plot(t, q(t), t, dq(t), t, ddq(t))

""" Single revolute joint / link """
def test_1R_robot_dynamics():
    l1 = 1.5
    m1 = 8
    Ig1 = 1/12 * m1 * l1**2
    
    def exact_dynamics(q, dq, ddq):
        return ( Ig1 + m1*(l1/2)**2 ) * ddq
    
    t = np.linspace(0, 10)
    t_exact = exact_dynamics(q(t), dq(t), ddq(t))
    
    r1 = Robot(['r'], [l1], [0])
    r1.set_link_inertia([m1], [l1/2], [Ig1])
    tau = np.zeros(t.shape)
    for i, ti in enumerate(t):
        tau[i] = r1.euler_newton([q(ti)], [dq(ti)], [ddq(ti)])
    
    np_test.assert_allclose(t_exact, tau)

""" Double link robot 2R """
#def test_2R_robot_dynamics():
#    r2 = Robot(['r', 'r'], [1.5, 1.5], [0, 0])
#    r2.set_link_inertia([8.0, 5.0], [0.75, 0.75], [8.0*1.5**2/12, 5.0*1.5**2/12])
#    
#    from exact_dynamics_2R import Exact_dynamics_2R
#    dyn = Exact_dynamics_2R(r2)
#    
#    t = np.linspace(0, 10)
#    t_exact = np.zeros((len(t), 2))
#    tau = np.zeros((len(t), 2))
#    
#    for i, ti in enumerate(t):
#        qi   = [q(ti), q(ti)]
#        dqi  = [dq(ti), dq(ti)]
#        ddqi = [ddq(ti), ddq(ti)]
#        t_exact[i] = dyn.run(qi, dqi, ddqi)
#        tau[i] = r2.euler_newton(qi, dqi, ddqi)
#    
#    np_test.assert_allclose(t_exact, tau)
    
    
#    fig, ax = plt.subplots()
#    r2.plot(ax, [0.3, 0.2])
#    
#    plt.figure()
#    plt.plot(tau)
#    plt.plot(t_exact)

if __name__ == "__main__":
    test_1R_robot_dynamics()
#    test_2R_robot_dynamics()


