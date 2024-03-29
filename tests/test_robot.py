#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from ppr.robot import Robot, Robot_3R, Robot_2P, Robot_2P3R

from ppr.robot import Robot, Robot_3R, Robot_2P, Robot_2P3R, RobotManyDofs

import numpy as np
import matplotlib.pyplot as plt
import pytest
from numpy.testing import assert_almost_equal, assert_

from ppr.geometry import Rectangle


class TestRobot:
    def test_get_shapes(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        s = robot1.get_shapes([np.pi / 2, -np.pi / 2])
        desired = [
            np.array(
                [
                    [0.0000000e00, 0.0000000e00],
                    [1.2246468e-16, 2.0000000e00],
                    [-5.0000000e-02, 2.0000000e00],
                    [-5.0000000e-02, 3.0616170e-18],
                ]
            ),
            np.array(
                [
                    [1.2246468e-16, 2.0000000e00],
                    [1.0000000e00, 2.0000000e00],
                    [1.0000000e00, 2.0500000e00],
                    [1.2246468e-16, 2.0500000e00],
                ]
            ),
        ]
        actual = s[0].get_vertices()
        assert_almost_equal(actual, desired[0])
        actual = s[1].get_vertices()
        assert_almost_equal(actual, desired[1])

    def test_wrong_joint_type(self):
        # check if exception is trown
        with pytest.raises(ValueError) as info:
            robot1 = Robot(["a", "r"], [2, 1], [0, 0])
            robot1.fk_all_links([0, 0])
        # check if the message was present
        msg = "wrong joint type. Must be 'r' or 'p', not: a"
        assert msg == str(info.value)

    def test_nonzero_base(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        robot1.base = [1, 2, np.pi / 2]
        actual = robot1.fk_all_links([0, np.pi / 2])
        desired = np.array([[1, 2, np.pi / 2], [1, 4, np.pi / 2], [0, 4, np.pi]])

        assert_almost_equal(actual, desired)

    def test_plot(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        robot1.plot(ax1, [0.1, 1.5])
        # plt.show()

    def test_plot_kinematics(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111)
        robot1.plot_kinematics(ax2, [0.1, 1.5])
        # plt.show()

    def test_plot_path(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        np.random.seed(42)
        path = np.random.randn(6, 2)
        robot1.plot_path(ax1, path)
        # plt.show()

    def test_plot_path_kinematics(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        fig1 = plt.figure()
        ax1 = fig1.add_subplot(111)
        np.random.seed(42)
        path = np.random.randn(6, 2)
        robot1.plot_path_kinematics(ax1, path)
        # plt.show()

    def test_set_joint_limits(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        robot1.set_joint_limits([(-1, 2), (-3, 2)])

    def test_set_link_inertia(self):
        robot1 = Robot(["r", "r"], [2, 1], [0, 0])
        robot1.set_link_inertia([1, 1], [0.5, 0.3], [0.05, 0.03])


class TestRobot_3R:
    def test_init_function(self):
        robot3r = Robot_3R([1.5, 1.0, 1.0])

    def test_init_input_error(self):
        with pytest.raises(ValueError) as info:
            robot3r = Robot_3R([1.5, 1.0])
        msg = "This robot has 3 links, not: 2"
        assert msg == str(info.value)

    def test_unreachable_inverse_kinematics(self):
        robot3r = Robot_3R([1.5, 1.0, 1.0])
        desired = {"success": False, "info": "unreachable"}
        p_unreachable = np.array([99.0, 5.0, 3.0])
        actual = robot3r.ik(p_unreachable)
        assert_(actual == desired)

    def test_special_inverse_kinematics(self):
        robot3r = Robot_3R([1, 1, 1])
        p1 = np.array([1, 0, np.pi])
        ik_sol = robot3r.ik(p1)
        ik_desired = np.array([[0, 0, np.pi]])
        assert_almost_equal(ik_sol["q"], ik_desired)

    def test_random_inverse_kinematics(self):
        np.random.seed(42)
        q_test = np.random.rand(10, 3)
        q_test = q_test * 2 * np.pi - np.pi
        robot3r = Robot_3R([1.5, 1.0, 1.0])
        for qi in q_test:
            p = robot3r.fk(qi)
            ik_sol = robot3r.ik(p)
            q_sol = ik_sol["q"]
            actual = [np.allclose(qj, qi) for qj in q_sol]
            assert_(np.any(actual))

    def test_self_collision(self):
        robot3r = Robot_3R([1, 1, 1])
        q1 = [0, 3, 3]
        robot3r.set_shapes_pose(q1)
        assert robot3r.check_self_collision() == True

        sc1 = []
        robot3r.do_check_self_collision = True
        assert robot3r.check_collision(q1, sc1) == True


class TestRobotDynamics:
    def test_fw_prop(self):
        pass


class TestRobot_2P:
    def test_init_function(self):
        robot2p = Robot_2P([1, 1])

    def test_init_input_error(self):
        with pytest.raises(ValueError) as info:
            robot3r = Robot_2P([1.5, 1.0, 1.0])
        msg = "This robot has 2 links, not: 3"
        assert msg == str(info.value)

    def test_random_inverse_kinematics(self):
        np.random.seed(42)
        q_test = np.random.rand(10, 2)
        q_test = q_test * 2 * np.pi - np.pi
        robot2p = Robot_2P([1.5, 1.0])
        for qi in q_test:
            p = robot2p.fk(qi)
            ik_sol = robot2p.ik(p)
            q_sol = ik_sol["q"]
            actual = [np.allclose(qj, qi) for qj in q_sol]
            assert_(np.any(actual))


class TestRobot_2P3R:
    def test_init_function(self):
        robot2p3r = Robot_2P3R([1.5, 1.0, 1.0, 0.5, 0.5])

    def test_init_input_error(self):
        with pytest.raises(ValueError) as info:
            robot2p3r = Robot_2P3R([1.5, 1.0])
        msg = "This robot has 5 links, not: 2"
        assert msg == str(info.value)

    def test_unreachable_inverse_kinematics(self):
        robot2p3r = Robot_2P3R([1.5, 1.0, 1.0, 0.5, 0.5])
        desired = {"success": False, "info": "unreachable"}
        p_unreachable = np.array([99.0, 5.0, 3.0])
        actual = robot2p3r.ik(p_unreachable)
        assert actual == desired

    def test_random_inverse_kinematics(self):
        np.random.seed(42)
        q_test = np.random.rand(10, 5)
        q_test = q_test * 2 * np.pi - np.pi
        # q_test[:, :2] = 0
        robot = Robot_2P3R([1.5, 1.0, 1.0, 0.5, 0.5])
        for qi in q_test:
            p = robot.fk(qi)
            ik_sol = robot.ik_fixed_joints(p, q_fixed=qi[0:2])
            q_sol = ik_sol["q"]
            actual = [np.allclose(qj, qi) for qj in q_sol]
            # assert_almost_equal(actual, [True, True])
            assert np.any(actual) == True

    def test_set_joint_limits(self):
        # joint limits only work for redundant joints at this moment
        robot2p3r = Robot_2P3R([1.5, 1.0, 1.0, 0.5, 0.5])
        robot2p3r.set_joint_limits([(0, 5), (0, 5), (), (), ()])
        desired = {"success": False, "info": "unreachable"}
        pose = np.array([3, 3, 3.0])
        actual = robot2p3r.ik(pose)
        actual_q = actual["q"]
        assert len(actual_q) == 6

    def test_fixed_link_shape(self):
        robot1 = Robot_2P3R([1, 1, 0.5, 0.5, 0.3])
        sc1 = [Rectangle(0.0, 0.4, 0.1, 0.2, -0.3), Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]
        q_collision = [0.17, 0.375, -2.02, 1.98, -1.03]
        robot1.set_shapes_pose(q_collision)
        shapes = robot1.collision_shapes
        results = []
        for recti in shapes:
            for rectj in sc1:
                results.append(recti.is_in_collision(rectj))
        assert np.any(results) == True


class TestRobotManyDofs:
    def test_init_function(self):
        r1 = RobotManyDofs(4)
        r2 = RobotManyDofs(10)

    def test_fk(self):
        r1 = RobotManyDofs(4, link_length=1.5)
        actual = r1.fk([0, 0, 0, 0])
        desired = np.array([4 * 1.5, 0, 0])
        assert_almost_equal(actual, desired)

        r2 = RobotManyDofs(10, link_length=1.5)
        actual = r2.fk([0] * 10)
        desired = np.array([10 * 1.5, 0, 0])
        assert_almost_equal(actual, desired)

    def test_ik(self):
        r1 = RobotManyDofs(4)
        pose1 = np.array([0, 2, np.pi / 2])
        sol = r1.ik_fixed_joints(pose1, [np.pi / 2])
        print(sol)
        desired = np.array([np.pi / 2, 0, 0, 0])
        assert_almost_equal(sol["q"][0], desired)
