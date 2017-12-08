#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Dec  8 10:16:09 2017

@author: jeroen
"""
from robot import Robot
from path import TolerancedNumber, TrajectoryPt
from geometry import Rectangle


def default_test_data():
    dx    = TolerancedNumber(1, 0.9, 1.1, samples=3)
    angle = TolerancedNumber(0.0, -0.5, 0.5, samples=5)
    path = []
    n_path = 10
    for i in range(n_path):
       yi = 0.7 + i * 0.6 / n_path
       path.append(TrajectoryPt([dx, yi, angle]))
    
    r1 = Robot(['r', 'r', 'r'], [1, 1, 0.5], [0, 0, 0])
    r1.set_link_inertia([1, 1, 1], [0.5, 0.5, 0.25], [0.05, 0.05, 0.05])
    
    sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
           Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
    
    return r1, path, sc1