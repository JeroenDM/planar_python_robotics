#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 28 11:34:35 2018

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
#from scipy.optimize import fmin_slsqp
from scipy.optimize import minimize

from ppr.geometry import Rectangle
from ppr.scene import plot_scene
from optimize2 import Problem

class Plane:
    def __init__(self):
        self.l = 1
        self.a = np.pi / 4
        self.x = 1.5
        self.y = 2.0
    
    def get_rectangles(self, q):
        return [Rectangle(q[0], q[1], self.l, self.l, self.a)]

plane = Plane()

water = [Rectangle(0.4, 0, 1, 1, np.pi / 4),
         Rectangle(2, 0, 3, 1, np.pi / 6)]

fig, ax = plt.subplots()
plane.get_rectangles([1.5, 2.0])[0].plot(ax)
plot_scene(ax, water)
ax.axis('equal')

def height(x):
    return x

# add collision contraints

# the default tolerance for slsqp is 1e-6 in the stopping criteria!
def create_collision_constraints(i, A2, b2, nvars, eps=1e-3):
    # nvars = number of variables in the primal problem
    lmin, lmax = nvars + i*8,     nvars + i*8 + 4
    mmin, mmax = nvars + i*8 + 4, nvars + i*8 + 8
    print(lmin, lmax)
    print(mmin, mmax)
    def distance_limit(z): 
        A1, b1 = plane.get_rectangles(z[:nvars])[0].get_matrix_form()
        la = z[lmin:lmax]
        mu = z[mmin:mmax]
        return -b1.dot(la) - b2.dot(mu) - eps
    
    def plane_equality(z):
        A1, b1 = plane.get_rectangles(z[:nvars])[0].get_matrix_form()
        la = z[lmin:lmax]
        mu = z[mmin:mmax]
        return A1.T.dot(la) + A2.T.dot(mu)
    
    def z_norm(z):
        A1, b1 = plane.get_rectangles(z[:nvars])[0].get_matrix_form()
        la = z[lmin:lmax]
        return -np.sum((A1.T.dot(la))**2) + 1
    
    c = []
    c.append({'type': 'ineq', 'fun': distance_limit})
    c.append({'type': 'ineq', 'fun': z_norm})
    c.append({'type': 'eq',   'fun': plane_equality})
    
    b = [(0, np.inf)] * 8
    return c, b

cons = []

A2, b2 = water[0].get_matrix_form()
cons2, bnds2 = create_collision_constraints(0, A2, b2, 2)

A3, b3 = water[1].get_matrix_form()
cons3, bnds3 = create_collision_constraints(1, A3, b3, 2)

cons = cons + cons2 + cons3
bounds = [(1.1, 1.9), (0, 6)] + bnds2 + bnds3

z0 = np.zeros(len(bounds))
#np.random.seed(13)
z0 = np.random.rand(len(bounds))
z0[0] = plane.x
z0[1] = plane.y

def obj(z):
    return height(z[1])

# solve problem
cons = tuple(cons)
sol = minimize(obj, z0, method='SLSQP', constraints=cons, bounds = bounds)
print(sol)

plane.get_rectangles(sol['x'][:2])[0].plot(ax)

las = sol['x'][1:5]
mus = sol['x'][5:9]