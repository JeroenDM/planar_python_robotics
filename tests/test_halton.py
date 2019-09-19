#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
from numpy.testing import assert_almost_equal
from ppr.halton import vdc, next_prime, HaltonSampler


def test_vdc():
    actual = [vdc(i) for i in range(1, 10)]
    desired = [0.5, 0.25, 0.75, 0.125, 0.625, 0.375, 0.875, 0.0625, 0.5625]
    assert_almost_equal(actual, desired)


def test_next_prime():
    pf1 = next_prime()
    pf2 = next_prime()
    a1 = [next(pf1) for i in range(5)]
    a2 = [next(pf2) for i in range(5)]
    assert_almost_equal(a1, a2)
    assert_almost_equal(a1, [3, 5, 7, 11, 13])


class TestHaltonSampler:
    def test_two_samplers(self):
        hs1 = HaltonSampler(3)
        hs2 = HaltonSampler(3)
        a1 = hs1.get_samples(5)
        d1 = np.array(
            [
                [0.33333333, 0.2, 0.14285714],
                [0.66666667, 0.4, 0.28571429],
                [0.11111111, 0.6, 0.42857143],
                [0.44444444, 0.8, 0.57142857],
                [0.77777778, 0.04, 0.71428571],
            ]
        )
        assert_almost_equal(a1, d1)
        a2 = hs2.get_samples(5)
        assert_almost_equal(a1, a2)

        a12 = hs1.get_samples(50)
        a22 = hs2.get_samples(50)
        assert_almost_equal(a12, a22)
