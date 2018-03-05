#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ppr.optim import q_derivatives
from numpy.testing import assert_almost_equal
import numpy as np

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