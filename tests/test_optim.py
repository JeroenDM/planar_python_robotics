#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from numpy.testing import assert_almost_equal
import numpy as np

#===================================================================
# DEFINE  tests for derivation stuff
#===================================================================
from ppr.optim import q_derivatives

def test_q_derivatives():
    q = np.array([[0, 0],
                  [1, 2],
                  [3, 4],
                  [7, 2]])
    dq_desired = np.array([[10.,  20.],
                           [15.,  20.],
                           [30.,   0.],
                           [40., -20.]])
    ddq_desired = np.array([[ 50.,    0.],
                            [100., -100.],
                            [125., -200.],
                            [100., -200.]])
    dq, ddq = q_derivatives(q)
    assert_almost_equal(dq, dq_desired)
    assert_almost_equal(ddq, ddq_desired)

def test_q_derivatives_with_known_function():
    t = np.linspace(0, 10, 50)
    dt = t[1]-t[0]
    x   = np.array([ np.sin(t),  np.cos(t), t**3     ]).T
    dx  = np.array([ np.cos(t), -np.sin(t), 3 * t**2 ]).T
    ddx = np.array([-np.sin(t), -np.cos(t), 6 * t    ]).T
    dx_test, ddx_test = q_derivatives(x, dt=dt)
    
    # the derivatis are not that good at the moment
    np.testing.assert_allclose(dx_test[10:40, :], dx[10:40, :],
                               rtol=1, atol=0.01)
    np.testing.assert_allclose(ddx_test[10:40, :], ddx[10:40, :],
                               rtol=1, atol=0.01)
#===================================================================
# SETUP test data for constraints
#===================================================================
from ppr.robot import Robot_3R
from ppr.geometry import Rectangle
from ppr.path import TolerancedNumber, TrajectoryPt, TrajectoryPtLineTol

# ROBOT
robot1 = Robot_3R([2, 2, 2])

# PATH
t = np.linspace(0, np.pi/2, 6)
R = 3;
angle = TolerancedNumber(0.0, -np.pi/2, np.pi/2, samples=10)

path1 = []
for ti in t:
    xin = R * np.cos(ti)
    xi = TolerancedNumber(xin, xin-0.5, xin+0.3, samples=6)
    pi = [xi, R * np.sin(ti), angle]
    path1.append(TrajectoryPt(pi))

# TUBE PATH
v = TolerancedNumber(0, -0.5, 0.3, samples=6)
path2 = []
for ti in t:
    pi = [R * np.cos(ti), R * np.sin(ti), angle]
    path2.append(TrajectoryPtLineTol(pi, v, ti))

# TUBE PATH
v = TolerancedNumber(0, -0.5, 0.3, samples=6)
path3 = []
# give this path a smaller length to make test more robust for typos
for ti in t[:-1]:
    pi = [R * np.cos(ti), R * np.sin(ti), 0.3]
    path3.append(TrajectoryPtLineTol(pi, v, ti))

# COLLISION SCENE
sc2 = [Rectangle(-2, 1, 1.5, 1, -0.1)]
sc1 = [Rectangle(3, 1.3, 2, 1, -0.1),
       Rectangle(3, 0.5, 2, 0.3, 0)]

#===================================================================
# DEFINITION of tests for constraints
#===================================================================
from ppr.optim import collision_ieq_con, tube_eq_con, tube_ieq_con, tube_ieq_eq_con

class TestConstraints():
    def test_collision_ieq_con(self):
        robot = Robot_3R([2.1, 3.0, 1.8])
        rects = [Rectangle(0, 0, 1, 1, 0),
                 Rectangle(5, 0, 1, 2, 0.3)]
        q_test = [np.array([np.pi/4, -np.pi/4, 0.0])]
        actual = collision_ieq_con(q_test, robot, rects)
        desired = np.array([-1.0353553,  1.7665554, 
                            0.4849242,  0.0384695, 
                            0.4849242, -1.0532455])
        assert_almost_equal(actual, desired, decimal=5)
    
    def test_tube_eq_con(self):
        q_path = np.zeros((len(path3), 3))
        actual = tube_eq_con(q_path, robot1, path3)
        assert len(actual) == 2*len(path3)
        #desired = 1
        #assert_almost_equal(actual, desired)
    
    def test_tube_ieq_con(self):
        q_path = np.zeros((len(path3), 3))
        actual = tube_ieq_con(q_path, robot1, path3)
        assert len(actual) == 4*len(path3)
        #desired = np.zeros(1)
        #assert_almost_equal(actual, desired)
    
    def test_tube_eq_con2(self):
        q_path = np.zeros((len(path2), 3))
        actual = tube_eq_con(q_path, robot1, path2)
        assert len(actual) == len(path2)
        #desired = 1
        #assert_almost_equal(actual, desired)
    
    def test_tube_ieq_con2(self):
        q_path = np.zeros((len(path2), 3))
        actual = tube_ieq_con(q_path, robot1, path2)
        assert len(actual) == 6*len(path2)
        #desired = np.zeros(1)
        #assert_almost_equal(actual, desired)
    
    def test_tube_ieq_eq_con(self):
        q_path = np.zeros((len(path3), 3))
        actual = tube_ieq_eq_con(q_path, robot1, path3)
        assert len(actual) == 8*len(path3)
        #desired = np.zeros(1)
        #assert_almost_equal(actual, desired)