# -*- coding: utf-8 -*-
"""s
Created on Mon Oct 23 16:59:16 2017

@author: jeroen
"""

def plot_scene(axes_handle, scene, *arg):
    for rect in scene:
        rect.plot(axes_handle, *arg)

""" testing """
if __name__ == "__main__":
    print("-----test scene.py-----")
    import numpy as np
    import matplotlib.pyplot as plt
    from geometry import Rectangle
    # generate random scene
    fig2 = plt.figure(2)
    ax2 = fig2.gca()
    plt.axis('equal')
    plt.axis([-10, 10, -10, 10])
    nr = 20
    r_pos = np.random.rand(nr, 2) * 10 - 5
    r_sha = np.random.rand(nr, 2) * 6 - 3
    r_ang = np.random.rand(nr) * np.pi - np.pi/2
    rectangles = []
    for i in range(nr):
        rectangles.append(Rectangle(r_pos[i, 0], r_pos[i, 1],
                                    r_sha[i, 0], r_sha[i, 1],
                                    r_ang[i]))
    plot_scene(ax2, rectangles, 'g')