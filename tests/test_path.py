#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ppr.path import TolerancedNumber, TrajectoryPt

import pytest
import numpy as np
import matplotlib.pyplot as plt
from numpy.testing import assert_almost_equal


class TestTolerancedNumber:
    def test_nominal_outside_bounds_error(self):
        with pytest.raises(ValueError) as info:
            a = TolerancedNumber(1.5, 0, 1)
        # check whether the error message is present
        msg = "nominal value must respect the bounds"
        assert msg in str(info)

    def test_get_initial_sampled_range(self):
        a = TolerancedNumber(2, 0, 4, samples=5)
        a1 = a.range
        d1 = [0, 1, 2, 3, 4]
        assert_almost_equal(a1, d1)


class TestTrajectoryPt:
    def test_plot(self):
        x = TolerancedNumber(1.5, 1.0, 2.0)
        y = TolerancedNumber(3.0, 2.9, 3.2)
        angle = TolerancedNumber(np.pi / 8, 0.0, np.pi / 4)
        tp = TrajectoryPt([x, y, angle])
        fig, ax = plt.subplots()
        plt.axis([0, 4, 0, 4])
        tp.plot(ax)
