#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from .path import cart_to_joint, get_shortest_path

class PlanningProblem():
    def __init__(self, path, robot, scene):
        self.path = path
        self.robot = robot
        self.scene = scene
    
    def ik(self, p):
        """ adapted inverse kinematics

        The redundant joints can be sampled different
        for every trajectory point. The sampling range can be set.
        """

    def single_run(self):
        Q = cart_to_joint(self.robot,
                          self.path,
                          check_collision=True,
                          scene=self.scene)
        return get_shortest_path(Q)