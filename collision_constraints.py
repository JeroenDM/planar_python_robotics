#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 09:49:52 2018

@author: jeroen
"""


import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fmin_slsqp

from ppr.geometry import Rectangle
from ppr.scene import plot_scene

# create list collision objects (Rectangles
sc1 = [Rectangle(1.0, 0.4, 0.1, 0.2, 0),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2),
       Rectangle(0.2, 0.8, 0.2, 0.2, 0)]

A1, b1 = sc1[0].get_matrix_form()
A2, b2 = sc1[1].get_matrix_form()


#%%
""" Minimal distance optimization problem """
fig3, ax3 = plt.subplots()
ax3.axis('equal')
ax3.axis([0, 1, 0, 1.5])
plot_scene(ax3, sc1, 'r')

# the primal formulation
def ieq_con(z):
    x = z[0:2]
    y = z[2:4]
    e1 = A1.dot(x) - b1
    e2 = A2.dot(y) - b2
    return -np.hstack((e1, e2))#.reshape((1,-1))

def obj(z):
    x = z[0:2]
    y = z[2:4]
    return np.sum((x - y)**2)

test_z = np.random.rand(4)
print(ieq_con(test_z))
print(ieq_con(test_z).shape)
print(obj(test_z))

ax3.plot(test_z[[0, 2]], test_z[[1, 3]], '*')


sol = fmin_slsqp(obj, test_z, f_ieqcons=ieq_con)

ax3.plot(sol[[0, 2]], sol[[1, 3]], 'g*')

#%%
""" the dual formulation """
#
def obj(z):
    la = z[0:4]
    mu = z[4:8]
    return b1.dot(la) + b2.dot(mu)

def eq_con2(z):
    la = z[0:4]
    mu = z[4:8]
    return A1.T.dot(la) + A2.T.dot(mu)

def ieq_con2(z):
    la = z[0:4]
    return -np.sum((A1.T.dot(la))**2) + 1

bounds = []
for i in range(8):
    bounds.append( (0, np.inf ) )
    

z0 = np.random.rand(8)
print(obj(z0))
print(eq_con2(z0))
print(ieq_con2(z0))
print(bounds)

sol = fmin_slsqp(obj, z0, f_eqcons=eq_con2,
                 f_ieqcons=ieq_con2,
                 bounds=bounds)


print(sol)

las = sol[0:4]
plane = A1.T.dot(las)
print( plane / np.sum( plane**2 ))


#%%
""" same dual problem, using a different interface to slsqp """
from scipy.optimize import minimize

def obj(z):
    la = z[0:4]
    mu = z[4:8]
    return b1.dot(la) + b2.dot(mu)

def eq_con2(z):
    la = z[0:4]
    mu = z[4:8]
    return A1.T.dot(la) + A2.T.dot(mu)


def ieq_con2(z):
    la = z[0:4]
    return -np.sum((A1.T.dot(la))**2) + 1

bounds = [(0, np.inf)] * 8

cons = []
cons.append({'type': 'ineq', 'fun': ieq_con2})
cons.append({'type': 'eq',   'fun': eq_con2})
cons = tuple(cons)
#print(cons)

z0 = np.random.rand(8)

sol = minimize(obj, z0, method='SLSQP', constraints = cons, bounds=bounds)

#sol = fmin_slsqp(obj, z0, f_eqcons=eq_con2,
#                 f_ieqcons=ieq_con2,
#                 bounds=bounds)
print(sol)
