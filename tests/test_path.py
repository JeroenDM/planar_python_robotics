#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pytest
import numpy as np
import matplotlib.pyplot as plt
from numpy.testing import assert_almost_equal

from ppr.path import TolerancedNumber, TrajectoryPt

class TestTolerancedNumber():
    def test_nominal_outside_bounds_error(self):
        with pytest.raises(ValueError) as info:
            a = TolerancedNumber(1.5, 0, 1)
        # check whether the error message is present
        msg = "nominal value must respect the bounds"
        assert(msg in str(info))

class TestTrajectoryPt():
    def test_plot(self):
        x = TolerancedNumber(1.5, 1.0, 2.0)
        y = TolerancedNumber(3.0, 2.9, 3.2)
        angle = TolerancedNumber(np.pi / 8, 0.0, np.pi / 4)
        tp = TrajectoryPt([x, y, angle])
        fig, ax = plt.subplots()
        plt.axis([0, 4, 0, 4])
        tp.plot(ax)