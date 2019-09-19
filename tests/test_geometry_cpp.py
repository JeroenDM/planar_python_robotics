#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 15:05:27 2018

@author: jeroen
"""

from ppr.cpp.geometry import Rectangle

import numpy as np
from scipy.linalg import norm
from scipy.optimize import minimize
from numpy.testing import assert_almost_equal, assert_, assert_allclose


class TestRectanlge:
    def test_init_function(self):
        rec1 = Rectangle(0.5, 2, 1, 3, 0)

    def test_check_is_in_collision(self):
        rec1 = Rectangle(1, 2, 3, 2, np.pi / 3)
        rec2 = Rectangle(1, 1, 3, 2, -np.pi / 3)
        actual = rec1.is_in_collision(rec1)
        desired = True
        assert_(actual == desired)

    def test_check_self_collision(self):
        rec1 = Rectangle(1, 2, 3, 2, np.pi / 3)
        actual = rec1.is_in_collision(rec1)
        desired = True
        assert_(actual == desired)

    def test_get_vertices(self):
        rec1 = Rectangle(0, 0, 1, 2, 0)
        actual = rec1.get_vertices()
        desired = np.array([[0, 0], [1, 0], [1, 2], [0, 2]])
        assert_almost_equal(actual, desired)

    def test_get_normals(self):
        rec1 = Rectangle(0, 0, 1, 2, 0)
        actual = rec1.get_normals()
        desired = np.array([[0, -1], [1, 0], [0, 1], [-1, 0]])
        assert_almost_equal(actual, desired)

    def test_set_pose(self):
        rec1 = Rectangle(1, 1, 1, 2, 0)
        rec1.set_pose(0, 2, np.pi / 2)
        actual = rec1.get_vertices()
        desired = np.array([[0, 2], [0, 3], [-2, 3], [-2, 2]])
        assert_almost_equal(actual, desired)
