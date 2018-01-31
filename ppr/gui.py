#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
from ppr.scene import plot_scene

from ppr.path import cart_to_joint, plot_path
from ppr.ga import get_shortest_path

# color scheme
col0 = 'w' # main background '#FFFFEA' # whiteish
col1 = '#D8D8D8' # widget_background (gray)
col2 = '#00CECB' # widget_foregroun (blueish)
# FF5E5B resish FFED66 greensih

class PlannerApp:
    """Gui object to execute and plot planning tasks """
    def __init__(self, output_axes, robot, scene, path):
        self.ax = output_axes
        self.r = robot
        self.s = scene
        self.p = path

class RobotApp:
    """Gui app to execute planning aglorithms"""
    def __init__(self, robot, q0, scene=None, path=None):
        self.robot = robot
        self.scene = scene
        self.path = path
        self.q0 = q0
        self.cc = False # check collision
        # create gui elements
        self.create_figure()
        self.draw_robot()
        self.draw_scene()
        self.draw_settings()
        self.draw_sliders()
        self.draw_button()
        self.draw_plan_button()
        # attach callbacks to events
        for s in self.sliders: s.on_changed(self.update)
        self.button.on_clicked(self.home)
        self.checks.on_clicked(self.toggle_cc)
        self.plan_button.on_clicked(self.sampling_planning)

    def create_figure(self):
        self.fig, self.ax_robot = plt.subplots(figsize=(8, 4), facecolor=col0)
        plt.title("The Robot")
        plt.subplots_adjust(left=0.5, bottom=0.1)
        self.ax_robot.axis([-2, 2, -2, 2])

    def draw_robot(self):
        self.robot.plot(self.ax_robot, self.q0, 'k')
        self.lines = self.ax_robot.get_lines()

    def draw_scene(self):
        if self.scene is not None:
            plot_scene(self.ax_robot, self.scene, 'g')

        self.scene_lines = []
        all_lines = self.ax_robot.get_lines()
        for l in all_lines:
            if l not in self.lines:
                self.scene_lines.append(l)

    def draw_settings(self):
        self.settings = self.fig.add_axes([0.1, 0.7, 0.3, 0.2])
        self.settings.text(0.1, 1.0, "Settings")
        self.checks = CheckButtons(self.settings, ("Check collision", "stuff"),
                                                    (False, True))


    def draw_sliders(self):
        self.ax_sliders = []
        self.sliders = []
        for i in range(self.robot.ndof):
            height = 0.1 + i * 0.1
            label = "joint {}".format(i)
            ax, sl = self.create_slider(self.fig, height, label, self.q0[i])
            self.ax_sliders.append(ax)
            self.sliders.append(sl)

    def draw_button(self):
        height = self.robot.ndof * 0.1 + 0.1
        resetax = self.fig.add_axes([0.1, height, 0.1, 0.05])
        self.button = Button(resetax, 'Home', color=col1, hovercolor='0.975')

    def start(self):
        plt.show()

    def home(self, event=None):
        """ Set sliders (and therefore robot) to home position """
        for s in self.sliders: s.reset()

    def update(self, val=None):
        q_new = [self.sliders[0].val, self.sliders[1].val, self.sliders[2].val]
        pt = [rec.get_plot_points() for rec in  self.robot.get_rectangles(q_new)]
        for i in range(len(self.lines)):
            self.lines[i].set_data(pt[i].T)

        if self.cc:
            if self.robot.check_collision(q_new, self.scene):
                self.set_scene_color('r')
            else:
                self.set_scene_color('g')

            self.fig.canvas.draw_idle()

    def set_scene_color(self, col):
        for l in self.scene_lines: l.set_color(col)

    def toggle_cc(self, label=None):
        if label == "Check collision":
            self.cc = not self.cc

    def draw_plan_button(self):
        height = self.robot.ndof * 0.1 + 0.1
        resetax = self.fig.add_axes([0.3, height, 0.1, 0.05], label="Plan")
        self.plan_button = Button(resetax, 'Plan', color=col1, hovercolor='0.975')

    def sampling_planning(self, event=None):
        if self.path is not None:
            self.log("Planning....")
            path_js = cart_to_joint(self.robot,
                                    self.path,
                                    check_collision = self.cc,
                                    scene = self.scene)
            res = get_shortest_path(path_js)
            if res['success']:
                self.log(res['path'])
                self.ax_robot.clear()
                self.ax_robot.axis([-2, 2, -2, 2])
                self.robot.plot_path(self.ax_robot, res['path'])
                plot_scene(self.ax_robot, self.scene)
                plot_path(self.ax_robot, self.path)
                self.log("Succes!")
            else:
                self.log("Planning failed")
        else:
            self.log("No path available")

    @staticmethod
    def create_slider(f, h, label, val_init):
        ax = f.add_axes([0.1, h, 0.3, 0.03], facecolor=col1, label=label)
        sl = Slider(ax, label, -3.14, 3.14, valinit=val_init,
                    color = col2)
        return ax, sl

    @staticmethod
    def log(msg):
        print(msg)


# ==== Slider control class ===
# class SliderControl:
#     def __init__(self, figure, x, y, ndof, init, w=0.3, h=0.03):
#         self.fig = figure
#         self.axes = []
#         self.sliders = []
#         for i in range(ndof):
#             label = "joint {}".format(i)
#             yi = y + 0.1 * i
#             self.add_ax([x, yi, w, h], facecolor=col1, label=label)
#             self.add_slider(self.axes[i], label, -3.14, 3.14, valinit=init[i], color = col2)
#
#     def add_ax(self, *arg, **kwarg):
#         self.axes.append(self.fig.add_axes(*arg, **kwarg))
#
#     def add_slider(self, *arg, **kwarg):
#         self.sliders.append(Slider(*arg, **kwarg))
# ===============================
