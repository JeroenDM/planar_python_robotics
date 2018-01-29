#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 10:16:01 2018

@author: jeroen
dummy problem

A rectangle must sink as low as possible in between two other rectangles
without collision. (But remain above the x-axis)
h = distance above x-axis
"""

import numpy as np
from numpy.random import rand
from scipy.optimize import minimize

class Problem:
    """ Setup and solve motion planning optimization problem """
    def __init__(self, robot, env, bounds = [], debug=False):
        self.robot = robot
        self.env = env
        self.cc = 0 # collision constraint counter
        self.nvars = robot.ndof
        
        self.constraints = []
        self.bounds = bounds
        
        self.debug = debug
    
    def solve(self, q_init):
        cons = tuple(self.constraints)
        bnds = self.bounds
        obj  = self.create_objective()
        
        z0 = rand(self.nvars + self.cc * 8)
        z0[:self.nvars] = q_init
        
        return minimize(obj, z0, method='SLSQP', constraints=cons, bounds = bnds)
    
    def create_objective(self):
        def obj(z):
            return z[1]
        return obj
    
    def add_collision_constraints(self):
        if len(self.bounds) == 0:
            # add default bounds
            print("Added default infinit bounds for joint variables")
            self.bounds = [(-np.inf, np.inf)] * self.nvars
        
        # loop over all collision objects in environement
        for rec in self.env:
            A2, b2 = rec.get_matrix_form()
            # loop over all robot link
            for i in range(self.robot.nlink):
                con, bound = self.create_collision_constraints(i, A2, b2)
                self.constraints += con
                self.bounds += bound
    
    def create_collision_constraints(self, link_index, A2, b2, eps=1e-3):
        self.cc += 1
        i = self.cc - 1
        nvars = self.nvars
        # nvars = number of variables in the primal problem
        lmin, lmax = nvars + i*8,     nvars + i*8 + 4
        mmin, mmax = nvars + i*8 + 4, nvars + i*8 + 8
        if self.debug:
            print(lmin, lmax)
            print(mmin, mmax)
        def distance_limit(z):
            A1, b1 = self.robot.get_rectangles(z[:nvars])[link_index].get_matrix_form()
            la = z[lmin:lmax]
            mu = z[mmin:mmax]
            return -b1.dot(la) - b2.dot(mu) - eps
        
        def plane_equality(z):
            A1, b1 = self.robot.get_rectangles(z[:nvars])[link_index].get_matrix_form()
            la = z[lmin:lmax]
            mu = z[mmin:mmax]
            return A1.T.dot(la) + A2.T.dot(mu)
        
        def z_norm(z):
            A1, b1 = self.robot.get_rectangles(z[:nvars])[link_index].get_matrix_form()
            la = z[lmin:lmax]
            return -np.sum((A1.T.dot(la))**2) + 1
        
        c = []
        c.append({'type': 'ineq', 'fun': distance_limit})
        c.append({'type': 'ineq', 'fun': z_norm})
        c.append({'type': 'eq',   'fun': plane_equality})
        
        b = [(0, np.inf)] * 8
        return c, b

from ppr.geometry import Rectangle
from ppr.scene import plot_scene
import matplotlib.pyplot as plt

class Plane:
    def __init__(self):
        self.l = 1
        self.a = np.pi / 4
        self.q0 = [1.0, 2.0]
        self.ndof = 2
        self.nlink = 2
    
    def get_rectangles(self, q):
        return [Rectangle(q[0], q[1], self.l, self.l, self.a),
                Rectangle(q[0] + 0.7, q[1] + 0.7, self.l, 0.1, 0),
                Rectangle(q[0] - 1.7, q[1] + 0.7, self.l, 0.1, 0)]

plane = Plane()

water = [Rectangle(0.4, 0, 1, 1, np.pi / 4),
         Rectangle(2, 0, 3, 1, np.pi / 6)]

fig, ax = plt.subplots()
for rec in plane.get_rectangles(plane.q0):
    rec.plot(ax, 'g')

plot_scene(ax, water)
ax.axis('equal')

opt = Problem(plane, water)#, bounds = [(1.1, 1.9), (0, 6)])

opt.add_collision_constraints()

sol = opt.solve(plane.q0)
print(sol)

for rec in plane.get_rectangles(sol['x'][:2]):
    rec.plot(ax, 'r')