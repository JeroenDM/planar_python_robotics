#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 15:55:23 2018

@author: jeroen
"""

import numpy as np
from scipy.optimize import minimize

def norm_sq(v):
    return np.sum(v**2)

def l1_norm(v):
    return np.sum(np.abs(dqp))

class Problem:
    """ wrapper for scipy.optimize.fmin_slsqp function """
    def __init__(self, robot, scene):
        self.robot = robot
        self.scene = scene
        self.nvars = robot.ndof
    
    def solve(self, x_init):
        return 0
    
    def joint_motion_obj(q_path):
        q_diff = np.diff(q_path, axis=0)
        return l1_norm(q_diff)


    
    

