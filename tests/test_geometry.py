#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from ppr.geometry import rotation, Rectangle
from numpy.testing import assert_allclose
import numpy as np

atol = 1e-15

def test_rotation():
    assert_allclose(rotation(0), np.eye(2))
    assert_allclose(rotation(np.pi / 2),
                    np.array([[0, -1], [1, 0]]),
                    atol=atol)

# create test rectangle known points
rec = Rectangle(0, 2, np.sqrt(2), np.sqrt(8), -np.pi/4)

def test_rectangle_get_plot_points():
    # calculate and check corner points
    res1 = rec.get_plot_points()
    sol1 = np.array([[0, 2], [1, 1], [3, 3], [2, 4]])
    assert_allclose(res1, sol1)

def test_rectangle_get_normals():
    res1 = rec.get_normals()
    s2 = np.sqrt(1/2)
    sol1 = np.array([[-s2, -s2], [s2, -s2], [s2, s2], [-s2, s2]])
    assert_allclose(res1, sol1)

def test_rectangle_project():
    """ This test fails, but in_collision seems to be workin..."""
    res1 = rec.project(np.array([0, -1]))
    sol1 = np.array([0, 1, 4, 3])
    assert_allclose(res1, sol1)

def test_rectangle_in_collision():
    rec2 = Rectangle(1, 2, 1, 0, 0)
    rec3 = Rectangle(5, 1, 1, 1, np.pi/6)
    assert rec.in_collision(rec2) == True
    assert rec.in_collision(rec3) == False