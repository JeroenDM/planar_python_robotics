#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 20 15:10:09 2017

@author: jeroen
"""
import numpy as np
import matplotlib.pyplot as plt

from example_robots import Robot_3R, Robot_2P3R

def create_axes_handle():
    fig = plt.figure()
    ah = fig.gca()
    plt.axis('equal')
    #plt.axis([-1, 3, -1, 3])
    return ah

def test_ik_3R():
    r = Robot_3R([0.5, 0.6, 0.3])
    r.set_base_pose([-1.1, 0.6, 0])
    N = 4
    qr = np.linspace(-np.pi, np.pi, N)
    grid = np.meshgrid(qr, qr, qr)
    grid = [ grid[i].flatten() for i in range(3) ]
    grid = np.array(grid).T
    
    q_test = grid
    #q_test = [[0, 0, 0], [0.5, 1.8, -1.3]]
    
    ax = create_axes_handle()
    for qi in q_test:
        pi = r.fk(qi)
        si = r.ik(pi)
        qs = []
        if si['success']:
            qs.append(si['q'])
            ax.plot(pi[0], pi[1], 'r*')
            for qj in si['q']:
                r.plot_kinematics(ax, qj, 'g')
            if np.allclose(qi, si['q'][0]) or np.allclose(qi, si['q'][1]):
                pass
            else:
                qpi = np.array([np.pi, np.pi, np.pi])
                if np.any(np.isclose(np.abs(qi), qpi)):
                    #edge case
                    pass
                else:
                    raise RuntimeError("Ik failed while comparing solutions " +
                                       str(qi) + "\nto\n" +
                                       str(si['q']))
        else:
            r.plot_kinematics(ax, qi, 'r')
            raise RuntimeError("Inverse kinematics failed to find solution")
    print("Test ik succeeded")

def test_ik_2P3R():
    r = Robot_2P3R([0.2, 0.2, 0.5, 0.6, 0.3])
    N = 4
    qr = np.linspace(-np.pi, np.pi, N)
    grid = np.meshgrid(qr, qr, qr)
    grid = [ grid[i].flatten() for i in range(3) ]
    grid = np.array(grid).T
    
    q_test = grid
    q_test = np.hstack(( np.random.rand(N**3, 2), q_test ))
    
#    q_test = [[0.0, 0, 0, 0.3, 0.5],
#              [0.1, 0, 0, 0.3, 0.5],
#              [0, 0, 0.5, 1.8, -1.3]]
#    q_test = np.array(q_test)
    
    ax = create_axes_handle()
    for qi in q_test:
        pi = r.fk(qi)
        si = r.ik(pi, q_fixed=qi[:2])
        qs = []
        if si['success']:
            qs.append(si['q'])
            ax.plot(pi[0], pi[1], 'r*')
            for qj in si['q']:
                r.plot_kinematics(ax, qj, 'g')
                if np.allclose(qi, si['q'][0]) or np.allclose(qi, si['q'][1]):
                    pass
                else:
                    qpi = np.array([0, 0, np.pi, np.pi, np.pi])
                    if np.any(np.isclose(np.abs(qi), qpi)):
                        #edge case
                        pass
                    else:
                        raise RuntimeError("Ik failed while comparing solutions " +
                                           str(qi) + "\nto\n" +
                                           str(si['q']))
        else:
            ax.plot(pi[0], pi[1], 'b*')
            r.plot_kinematics(ax, qi, 'r')
            raise RuntimeError("Inverse kinematics failed to find solution")
    print("Test ik succeeded")

if __name__ == "__main__":
    test_ik_3R()
    test_ik_2P3R()