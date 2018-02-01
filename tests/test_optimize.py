
import numpy as np
import matplotlib.pyplot as plt

from ppr.geometry import Rectangle
from ppr.scene import Scene
from ppr.path import sampled_data
from ppr.optimize import Problem

class Car:
    """
    Mimics the Robot class from ppr.robot
    box shape that can move around along x and y
    and turn around its corner
    Assuming forward kinmatics x, y, a = q0, q1, q2
    """
    def __init__(self, width=1, length=1.5):
        self.w = width
        self.l = length
        q0 = [0, 0, 0] # [x, y, angle]
        self.q0 = q0
        self.shape = Rectangle(q0[0], q0[1], width, length, q0[2])
        self.ndof = 3
        self.nlink = 1

    def move(self, q):
        """ Create new shape at the new car's position """
        self.shape = Rectangle(q[0], q[1], self.w, self.l, q[2])

    def fk(self, q):
        return q

    def get_rectangles(self, q):
        self.move(q)
        return [self.shape]

    def plot(self, axes_handle, q, *arg, **kwarg):
        self.move(q)
        self.shape.plot(axes_handle, *arg, **kwarg)

    def plot_path(self, axes_handle, qp):
        """ Plot a list of joint positions """
        # draw robot more transparant to the end of the path
        alpha = np.linspace(1, 0.2, len(qp))
        for i, qi in enumerate(qp):
            self.plot(axes_handle, qi, color=(0.1, 0.2, 0.5, alpha[i]))


car = Car()
scene = Scene([Rectangle(0, 2, 1, 1.5, -0.1),
               Rectangle(2, 2, 1, 2, 0),
               Rectangle(4.5, 2.5, 1, 3, -0.5)])
field = Rectangle(0, 0, 7, 7, 0)

# initial path to go from (0, 0, 0) to (0, 6, 0)
N = 10
nvars = N * car.ndof
qp0 = np.zeros((N, 3))
qp0[:, 1] = np.linspace(0, 6, N)
qp0[:, 0] = 1

""" plotting """
fig, ax = plt.subplots()
# car.plot(ax, [0, 0, 0])
# car.plot_path(ax, qp0)
scene.plot(ax, 'g')
field.plot(ax, 'k')
# plt.axis('equal')
# plt.show()

from ppr.path import TrajectoryPt, TolerancedNumber, plot_path

dx = TolerancedNumber(0.0, 0.0, 7.0)
da = TolerancedNumber(0.0, -np.pi/2, np.pi/2)
path = [TrajectoryPt([dx, yi, da]) for yi in np.linspace(0, 6, N)]

plot_path(ax, path, show_tolerance=False)


joint_limits = [(0, 7), (0, 7), (-np.pi/2, np.pi/2)]
opt = Problem(car, scene.shapes, path)
opt.add_joint_limits(joint_limits)
opt.add_path_constraints()
opt.add_collision_constraints()
sol = opt.solve(qp0.flatten())
print(sol)

if sol['success']:
    p_sol = sol['x'][:nvars]
    p_sol = p_sol.reshape(N, 3)
    car.plot_path(ax, p_sol)
    plt.show()
else:
    print("Optimization not successful")
