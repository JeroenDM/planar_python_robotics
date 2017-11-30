#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 21 11:59:33 2017

@author: jeroen
"""

from scipy.interpolate import CubicSpline, interp1d
import numpy as np
from scipy.signal import resample

def derivatives(t, x):
    """ Calculate derivative along second axis,
    or first axis if one dimensional input
    """
    
    x = np.array(x)
    sh = x.shape
    if len(sh) == 1:
        N = sh[0]
        M = 1
    else:
        M, N = sh
    
    Ns = 30
    ts = np.linspace(0, t[-1], Ns)
    f = interp1d(t, x)
    xs = f(ts)
    print(xs.shape)
    sp = []
    for i in range(M):
        cs = CubicSpline(ts, xs[i], bc_type='clamped')
        sp.append(cs)
    return ts, xs, sp

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    #x = np.arange(10)
    x = np.linspace(0, 10, 10)
    y = np.sin(x)
    cs = CubicSpline(x, y)
    xs = np.arange(-0.5, 9.6, 0.1)
    plt.figure(figsize=(6.5, 4))
    plt.plot(x, y, 'o', label='data')
    plt.plot(xs, np.sin(xs), label='true')
    plt.plot(xs, cs(xs), label="S")
    plt.plot(xs, cs(xs, 1), label="S'")
    plt.plot(xs, cs(xs, 2), label="S''")
    plt.plot(xs, cs(xs, 3), label="S'''")
    plt.xlim(-0.5, 9.5)
    plt.legend(loc='lower left', ncol=2)
    plt.show()
    
    t = np.linspace(0, 10, 10)
    q = np.array([y, y, -y])
    ts, xs, s = derivatives(t, q)
    
    fig5, ax5 = plt.subplots()
    ax5.plot(ts, xs[0], '.')
    qs = s[0](ts)
    dq = s[0](ts, 1)
    ddq = s[0](ts, 2)
    ax5.plot(ts, qs, ts, dq, ts, ddq)