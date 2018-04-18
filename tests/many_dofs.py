#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 18 18:56:42 2018

@author: jeroen
"""

import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import RobotManyDofs

#r1 = RobotManyDofs(4)
#pose1 = np.array([0, 2, np.pi / 2])
#sol = r1.ik_fixed_joints(pose1, [np.pi/2])
#
#print(sol)
#
#
#
#fig, ax = plt.subplots()
#for qi in sol['q']:
#    r1.plot(ax, qi, 'k')
#
#plt.show()


r2 = RobotManyDofs(8)
qs = r2.sample_redundant_joints(10)
print(qs)

pose1 = np.array([0, 2, np.pi / 2])