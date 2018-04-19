#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ppr.path import vdc, vdc_generator
from ppr.path import TolerancedNumber, TrajectoryPt

import pytest
import numpy as np
import matplotlib.pyplot as plt
from numpy.testing import assert_almost_equal

def test_vdc():
    actual = [vdc(i) for i in range(1, 10)]
    desired = [0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, 0.0625, 0.5625]
    assert_almost_equal(actual, desired)

def test_vdc_generator():
    gens = [vdc_generator(-3, -2),
            vdc_generator(-1, 1.5),
            vdc_generator(2.1, 4)]
    desired = [
            [-2.5, -2.75, -2.25, -2.875, -2.375, -2.625, -2.125, -2.9375,
             -2.4375, -2.6875],
            [0.25, -0.375, 0.875, -0.6875, 0.5625, -0.0625, 1.1875, -0.84375,
             0.40625, -0.21875],
            [3.05, 2.575, 3.525, 2.3375, 3.2875, 2.8125, 3.7625, 2.21875,
             3.16875, 2.69375]
            ]
    for i in range(len(gens)):
        r = [gens[i].__next__() for j in range(10)]
        assert_almost_equal(r, desired[i])

class TestTolerancedNumber():
    def test_nominal_outside_bounds_error(self):
        with pytest.raises(ValueError) as info:
            a = TolerancedNumber(1.5, 0, 1)
        # check whether the error message is present
        msg = "nominal value must respect the bounds"
        assert(msg in str(info))
    
    def test_get_initial_sampled_range(self):
        a = TolerancedNumber(2, 0, 4, samples=5)
        a1 = a.range
        d1 = [ 0, 1, 2, 3, 4]
        assert_almost_equal(a1, d1)

class TestTrajectoryPt():
    def test_plot(self):
        x = TolerancedNumber(1.5, 1.0, 2.0)
        y = TolerancedNumber(3.0, 2.9, 3.2)
        angle = TolerancedNumber(np.pi / 8, 0.0, np.pi / 4)
        tp = TrajectoryPt([x, y, angle])
        fig, ax = plt.subplots()
        plt.axis([0, 4, 0, 4])
        tp.plot(ax)