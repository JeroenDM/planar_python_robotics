#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 12:57:03 2017

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin_slsqp
from scipy.interpolate import CubicSpline
np.random.seed(4)

def coef_to_spline(coef, x):
    """ convert a 1D numpy array to a spline with these values as coefs  """
    if len(coef) % 4 == 0:
        N = int(len(coef) / 4)
    else:
        raise ValueError("Length of coef must be multiple of 4.")
    
    y = np.zeros(x.shape) # dummy y values to create spline
    sp = CubicSpline(x, y, bc_type='clamped')
    sp.c = coef.reshape(4, N).copy()
    return sp

def points_to_spline(points, x):
    """ convert a 1D numpy array to a spline with these values as coefs  """
    sp = CubicSpline(x, points, bc_type='clamped')
    return sp

def integrate_spline(sp):
    xe = np.linspace(sp.x[0], sp.x[-1], 500)
    return np.sum(sp(xe)) * (xe[1] - xe[0])

x = np.linspace(0, 1, 11)
y = np.random.rand(11)

c = CubicSpline(x, y, bc_type='clamped')
c2 = coef_to_spline( (c.c + 0.1).flatten(), x)
rcoef = np.random.rand(4, 10)
c3 = coef_to_spline(rcoef.flatten(), x)

xs = np.linspace(0, 1, 100)
plt.figure()
plt.plot(x, y, '.')
plt.plot(xs, c(xs))
plt.plot(xs, c2(xs))
plt.plot(xs, c3(xs))


def obj(var, x):
    sp = points_to_spline(var, x)
    return integrate_spline(sp)

def ieq(var, x):
    sp = points_to_spline(var, x)
    xe = np.linspace(sp.x[0], sp.x[-1], 30)
    x0 = sp(xe) + 3
    x1 = np.array([sp(0.5) - 2,
                   sp(0.2) - 1,
                   sp(0.9) - 0.5])
    return np.concatenate((x0, x1))

var_test = np.zeros(11)
#print( obj(var_test, x) )
#print( ieq(var_test, x) )
plt.plot(x[5], c(x[5]), '*')

cinit = np.zeros(11)
sol = fmin_slsqp(lambda v: obj(v, x),
                 cinit,
                 f_ieqcons = lambda v: ieq(v, x))

c_sol = points_to_spline(sol, x)
plt.figure()
plt.plot(x, c_sol(x), 'o')
plt.plot(xs, c_sol(xs), '.')
plt.plot([0.5, 0.2, 0.9], [2, 1, 0.5], 'r^')
plt.plot(x, -3 * np.ones(x.shape), 'r')

#ct = CubicSpline(x, c_sol(x), bc_type='clamped')
#plt.plot(xs, ct(xs))