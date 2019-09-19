#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 11 19:01:03 2018

@author: jeroen
"""

import matplotlib.pyplot as plt
import numpy as np
from numpy.random import randint, rand

from ppr.geometry import Rectangle


def create_random_scene(n, xlim=[-10, 10], ylim=[-10, 10]):
    x_pos = rand(n) * (xlim[1] - xlim[0]) + xlim[0]
    y_pos = rand(n) * (ylim[1] - ylim[0]) + ylim[0]
    dx = rand(n) * 6 - 3
    dy = rand(n) * 6 - 3
    a = rand(n) * np.pi - np.pi / 2
    return [Rectangle(x_pos[i], y_pos[i], dx[i], dy[i], a[i]) for i in range(n)]


s = create_random_scene(10)

fig, ax = plt.subplots()
for r in s:
    r.plot(ax, "g")
plt.show()
