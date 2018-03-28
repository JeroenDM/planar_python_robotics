#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

#===================================================================
# SETUP test data
#===================================================================
from ppr.robot import Robot_3R
from ppr.geometry import Rectangle
from ppr.path import TolerancedNumber, TrajectoryPt, TrajectoryPtLineTol

# ROBOT
robot1 = Robot_3R([2, 2, 2])

# PATH
dx    = np.linspace(3, 4, 10)
dy    = TolerancedNumber(1.0, 1.0, 1.1, samples=5)
angle = TolerancedNumber(0.0, -np.pi/2, np.pi/2, samples=10)
path1 = [TrajectoryPt([xi, dy, angle]) for xi in dx]

# COLLISION SCENE
sc1 = [Rectangle(3, 1.3, 2, 1, -0.1),
       Rectangle(3, 0.5, 2, 0.3, 0)]

#===================================================================
# DEFINE tests
#===================================================================
from ppr.planning import PlanningProblem

class TestPlanningProblem():
    def test_init(self):
        PlanningProblem(path1, robot1, sc1)
    
    def test_single_run(self):
        pp = PlanningProblem(path1, robot1, sc1)
        sol = pp.single_run()
        assert sol['success'] == True
        assert len(sol['path']) == len(path1)