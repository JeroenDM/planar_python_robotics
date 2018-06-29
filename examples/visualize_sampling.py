#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 17 11:20:29 2018

@author: jeroen
"""
import numpy as np
from numpy import cos, sin, pi
import matplotlib.pyplot as plt

def create_grid(r):
    grid = np.meshgrid(*r)
    grid = [ grid[i].flatten() for i in range(len(r)) ]
    grid = np.array(grid).T
    return grid


class Robot():
    def __init__(self, l1, l2):
        self.ll = [l1, l2]
    
    def fk(self, q):
        l1, l2 = self.ll
        x = l1 * cos(q[0]) + l2 * cos(q[0] + q[1])
        y = l1 * sin(q[0]) + l2 * sin(q[0] + q[1])
        return np.array([x, y])

robot = Robot(1, 0.5)
N = 15
#q1 = np.linspace(-pi, pi, N)
#q2 = np.linspace(-pi, pi, N)
#q_samples = create_grid([q1, q2])

q_samples = np.random.rand(N*N, 2) * 2 * pi - pi
q_samples[:, 0] = q_samples[:, 0] / 2
qs2 = np.random.rand(N*N, 2) * 2 * pi - pi

xy = np.array([robot.fk(q) for q in q_samples])

plt.subplot(121)
plt.plot(q_samples[:, 0], q_samples[:, 1], 'o',
         qs2[:, 0], qs2[:, 1], '.')
plt.axis('equal')

plt.subplot(122)
plt.plot(xy[:, 0], xy[:, 1], '.')
plt.axis('equal')

plt.show()