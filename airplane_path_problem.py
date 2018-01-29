#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 10:41:08 2018

@author: jeroen
"""

""" setup problem """
import numpy as np
import matplotlib.pyplot as plt
from ppr.geometry import Rectangle
from ppr.scene import plot_scene
from ppr.path import TolerancedNumber, TrajectoryPt, plot_path

class Plane:
    def __init__(self):
        self.l = 1
        self.a = np.pi / 4
        self.q0 = [0.0, 0.0]
        self.ndof = 2
        self.nlink = 3
    
    def ik(self, p):
        return {'success': True, 'q': [[ p[0], p[1] ]]}
    
    def fk(self, q):
        return np.array([q[0], q[1], 0.0])
        
    
    def check_collision(self, q, other_rectangles):
        """ Check for collision between the robot other_rectangles """
        
        for recti in self.get_rectangles(q):
            for rectj in other_rectangles:
                if recti.in_collision(rectj):
                    return True # exit if a collision is found
        return False
    
    def get_rectangles(self, q):
        return [Rectangle(q[0], q[1], self.l, self.l, self.a),
                Rectangle(q[0] + 0.7, q[1] + 0.7, 0.5, 0.1, 0),
                Rectangle(q[0] - 1.2, q[1] + 0.7, 0.5, 0.1, 0)]

    def plot(self, axes_handle, q, *arg):
        for rec in self.get_rectangles(q):
            rec.plot(axes_handle, *arg)
    
    def plot_path(self, axes_handle, q_path):
        for q in q_path:
            self.plot(axes_handle, q, 'k')

plane = Plane()

water = [Rectangle(2, 0, 1, 1, np.pi / 4),
         Rectangle(2.5, 1, 0.5, 0.3, np.pi / 6)]

dy    = TolerancedNumber(0, 0, 5, samples=5)
n_path = 6
path = [TrajectoryPt([i*6 / n_path, dy, 0]) for i in range(n_path)]
path[0]  = TrajectoryPt([0.0, 0.0, 0.0])
path[-1] = TrajectoryPt([5.0, 0.0, 0.0])

fig, ax = plt.subplots()
plane.plot(ax, plane.q0, 'g')
plot_scene(ax, water)
ax.axis('equal')
plot_path(ax, path)

""" graph solve """
from ppr.path import cart_to_joint
# calculate all possible (collision free) joint configurations
# for each path point
path_js = cart_to_joint(plane, path, check_collision = True, scene = water)
# modify path
Q = path_js
#Q[0] = Q[0][0].reshape((1, plane.ndof))
#Q[-1] = Q[-1][0].reshape((1, plane.ndof))

#from ppr.cpp.graph_cpp import get_shortest_path
#from ppr.graph import get_shortest_path
np.random.seed(3)
from ppr.ga import get_shortest_path

# cpp version usen only one start node at the moment

# find the best sequence of joint solutions in path_js
# currently total joint movement is minimized by default
res = get_shortest_path(path_js)
if res['success']:
    Qp = res['path']
    #path_length = res['length']
    fig4, ax4 = plt.subplots()
    plt.title("The first solution")
    ax4.axis('equal')
    plane.plot_path(ax4, Qp)
    plot_path(ax4, path, show_tolerance=False)
    plot_scene(ax4, water, 'r')
    #plt.savefig("image/example_first_solution.png")
else:
    print("no path found")

""" now the real optimization """
from ppr.optimize import Problem

# joint limits as bounds for the optimization problem
joint_limits = [(0, 6), (0, 6)] * n_path

opt = Problem(plane, water, path, bounds=joint_limits)
opt.add_path_constraints()

#q0 = np.zeros(opt.nvars)
#q0 = np.random.rand(opt.nvars)
q0 = np.array(Qp).flatten()
#for c in opt.constraints: print(c['fun'](z0))

opt.add_collision_constraints()

sol = opt.solve(q0)
print(sol)

if sol['success']:
    Qs = sol['x']
    Qs = Qs[:opt.nvars] # remove dual variables
    Qs = Qs.reshape(n_path, -1)
    fig5, ax5 = plt.subplots()
    plt.title("Optimal solution?")
    ax5.axis('equal')
    plane.plot_path(ax5, Qs)
    plot_path(      ax5, path, show_tolerance=False)
    plot_scene(     ax5, water, 'r')
    #plt.savefig("image/example_first_solution.png")
else:
    print("no optimal path found")
