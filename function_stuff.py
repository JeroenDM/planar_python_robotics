#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jan 27 17:45:08 2018

@author: jeroen
"""

import numpy as np

def f1(x):
    print("f1 executed")
    return x + 2

f3 = 

def f2(z):
    x = z[0]
    y = z[1]
    return f1(x) + y

print(f1(2))
print(f2([2, 0.5]))