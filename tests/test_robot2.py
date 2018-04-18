#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from numpy.testing import assert_almost_equal
import pytest
from ppr.robot import RobotManyDofs

class TestRobotManyDofs():
    def test_init_function(self):
        r1 = RobotManyDofs(4)
        r2 = RobotManyDofs(10)
    
    def test_fk(self):
        r1 = RobotManyDofs(4, link_length=1.5)
        actual = r1.fk([0, 0, 0, 0])
        desired = np.array([4*1.5, 0, 0])
        assert_almost_equal(actual, desired)
        
        r2 = RobotManyDofs(10, link_length=1.5)
        actual = r2.fk([0]*10)
        desired = np.array([10*1.5, 0, 0])
        assert_almost_equal(actual, desired)
    
    def test_ik(self):
        r1 = RobotManyDofs(4)
        pose1 = np.array([0, 2, np.pi / 2])
        sol = r1.ik_fixed_joints(pose1, [np.pi/2])
        print(sol)
        desired = np.array([np.pi/2, 0, 0, 0])
        assert_almost_equal(sol['q'][0], desired)