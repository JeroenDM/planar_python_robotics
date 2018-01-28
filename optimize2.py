#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 15:55:23 2018

@author: jeroen
"""

import numpy as np
from scipy.optimize import fmin_slsqp

def norm_sq(v):
    return np.sum(v**2)

class Problem:
    """ wrapper for scipy.optimize.fmin_slsqp function """
    def __init__(self, robot, obj=None, eq=None, ieq=None, bounds=[]):
        self.robot = robot
        self.obj = obj
        self.eq = eq
        self.ieq = ieq
        self.bounds = bounds
        self.nvars = 1
        self.eps = 1e-6
    
    def solve(self, x_init):
        return fmin_slsqp(self.obj, x_init,
                          f_eqcons=self.eq,
                          f_ieqcons=self.ieq)
    
    def add_collision_constraint(self, A, b):
        # wrap objective with longer variable vector
        A2 = A; b2 = b;
        
        def ieq(z):
            n = self.nvars
            # old stuff
            h = z[n-1]
            if self.ieq == None:
                ie0 = None
            else:
                ie0 = self.ieq(h)
            
            # new stuff
            la = z[n:(n + 4)]
            mu = z[(n + 4):(n + 8)]
            A1, b1 = self.robot.get_rectangles(h)[0].get_matrix_form()
            ie1 = -norm_sq(A1.dot(la)) + 1
            ie2 = b1.dot(la) + b2.dot(mu)
            
            return np.hstack((ie1, ie2))
        
        def eq(z):
            h = z[0]
            la = z[1:5]
            mu = z[5:9]
            A1, b1 = self.robot.get_rectangles(h)[0].get_matrix_form()
            
            return A1.T.dot(la) + A2.T.dot(mu)
        
        
        self.bounds = self.bounds + [(0, np.inf)] * 8
        
            
            


#def create_objective(A1, A2, b1, b2):
#    def obj(z):
#        la = z[0:4]
#        mu = z[4:8]
#        return b1.dot(la) + b2.dot(mu)
#    return obj
#
#def create_inequality_constraints(A1, A2, b1, b2):
#    def ieq(z):
#        la = z[0:4]
#        return -np.sum((A1.T.dot(la))**2) + 1
#    return ieq
#
#def create_equality_constraints(A1, A2, b1, b2):
#    def eq(z):
#        la = z[0:4]
#        mu = z[4:8]
#        return A1.T.dot(la) + A2.T.dot(mu)
#    return eq
#
#def create_bounds():
#    return [(0, np.inf )] * 8


