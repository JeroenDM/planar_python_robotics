#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 21 18:28:43 2018

@author: jeroen
"""

import matplotlib.pyplot as plt

from ppr.geometry import Rectangle
from ppr.scene import plot_scene

shapes = [Rectangle(1, 1, 2, 2, 0),
          Rectangle(4, 1, 3, 2, 0)]

fig, ax = plt.subplots()
plot_scene(ax, shapes)

A1, b1 = shapes[0].get_matrix_form()
A2, b2 = shapes[1].get_matrix_form()

print(A1, b1, A2, b2)

