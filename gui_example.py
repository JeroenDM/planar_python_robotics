#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np

from ppr.gui import RobotApp
from example_robots import Robot_3R
from ppr.cpp.geometry_cpp import Rectangle

robot3R = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])
sc1 = [Rectangle(0.2, 0.4, 0.1, 0.2, -0.3),
       Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
q_init = [1.5, 0.0, 0.0]

app = RobotApp(robot3R, q_init, scene=sc1)
app.start()


# fig, ax_robot = plt.subplots(facecolor=main_background)
# plt.title("The Robot")
# plt.subplots_adjust(left=0.3, bottom=0.25)
# ax_robot.axis([-2, 2, 0, 3])
# # ax_robot.axis('equal')
#
#
# q0 = [1.5, 0.0, 0.0]
#
# def plot_fun(ax, data):
#     ax.plot([-1, 0, 1], data, '*')
#
#
# robot.plot(ax_robot, q0, 'k')
# lines = ax_robot.get_lines()
#
# axcolor = 'lightgoldenrodyellow'
#
# ax_sliders = []
# sliders = []
# h_max = 0.2
# for i in range(robot.ndof):
#     label = "s_{}".format(i)
#     h = i * 0.1
#     joint_label = "joint {}".format(i)
#     ax_sliders.append(plt.axes([0.2, h, 0.6, 0.03],
#                             facecolor=widget_background,
#                             label=label))
#     sliders.append(Slider(ax_sliders[i],
#                             joint_label,
#                             0.0, 3.14,
#                             valinit=q0[0],
#                             color = widget_foreground))
#
# def update(val):
#     q_new = [sliders[0].val, sliders[1].val, sliders[2].val]
#     pt = [rec.get_plot_points() for rec in  robot.get_rectangles(q_new)]
#     for i in range(len(lines)):
#         lines[i].set_data(pt[i].T)
#     # lines[0].set_ydata(q_new)
#     # plot_fun(ax_robot, q_new)
#     fig.canvas.draw_idle()
#
# for s in sliders: s.on_changed(update)
#
# # robot1 = Robot_3R([1, 1, 0.5], [0.05, 0.03, 0.02])
#
# # plot the robot in some configuration
# # fig2, ax2 = plt.subplots()
# # plt.title("The Robot")
# # ax2.axis('equal')
# # robot1.plot(ax2, [1.3, -0.8, 1.4], 'k')
#
# plt.show()

# fig, ax = plt.subplots()
# plt.subplots_adjust(left=0.25, bottom=0.25)
# t = np.arange(0.0, 1.0, 0.001)
# a0 = 5
# f0 = 3
# s = a0*np.sin(2*np.pi*f0*t)
# l, = plt.plot(t, s, lw=2, color='red')
# plt.axis([0, 1, -10, 10])
#
# axcolor = 'lightgoldenrodyellow'
# axfreq = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor=axcolor)
# axamp = plt.axes([0.25, 0.15, 0.65, 0.03], facecolor=axcolor)
#
# sfreq = Slider(axfreq, 'Freq', 0.1, 30.0, valinit=f0)
# samp = Slider(axamp, 'Amp', 0.1, 10.0, valinit=a0)
#
#
# def update(val):
#     amp = samp.val
#     freq = sfreq.val
#     l.set_ydata(amp*np.sin(2*np.pi*freq*t))
#     fig.canvas.draw_idle()
# sfreq.on_changed(update)
# samp.on_changed(update)
#
# resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
# button = Button(resetax, 'Reset', color=axcolor, hovercolor='0.975')
#
#
# def reset(event):
#     sfreq.reset()
#     samp.reset()
# button.on_clicked(reset)
#
# rax = plt.axes([0.025, 0.5, 0.15, 0.15], facecolor=axcolor)
# radio = RadioButtons(rax, ('red', 'blue', 'green'), active=0)
#
#
# def colorfunc(label):
#     l.set_color(label)
#     fig.canvas.draw_idle()
# radio.on_clicked(colorfunc)
#
# plt.show()
