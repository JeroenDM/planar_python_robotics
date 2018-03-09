#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from numpy.testing import assert_almost_equal
from ppr.ga import GASolver, get_shortest_path_ga, LazyGASolver
from ppr.robot import Robot_2P3R
from ppr.path import TolerancedNumber, TrajectoryPt
from ppr.geometry import Rectangle
from ppr.util import VirtualJointSolutions

data = [np.array([[0, 0], [0, 1]]),
        np.array([[1, -1], [1, 0], [1, 1]]),
        np.array([[0, 2], [2, 2]])]
data_size = [len(q) for q in data]

solver = GASolver(data)

class TestLazyGASolver():
    def test_create_chromosome(self):
        # create trajectory point
        x = TolerancedNumber(2, 1, 3, samples=3)
        y = TolerancedNumber(2.5, 2, 3, samples=4)
        tp = TrajectoryPt([x, y, 0])
        
        # create robot and collision scene
        robot1 = Robot_2P3R([1, 1, 2, 1.5, 0.5])
        sc1 = [Rectangle(2, 1, 0.5, 0.5, 0)]
        
        # create the object
        Qv = [VirtualJointSolutions(robot1, tp, sc1),
              VirtualJointSolutions(robot1, tp, sc1)]
        
        ls = LazyGASolver(Qv)
        ch1, path1 = ls.create_chromosome()
        assert len(ch1) == len(Qv)
        assert ch1[0] in range(len(Qv[0]))
        assert ch1[1] in range(len(Qv[1]))
        
        # the number of samples is 3*4 (path) *5*5 (ik) * 2 (configurations)
        # = 600
        assert len(Qv[0]) == 3*4*5*5*2
        
        # check path dimensions
        x, y = path1.shape
        assert x == len(Qv)
        assert y == robot1.ndof
    
    def test_create_population(self):
        # default size value
        # create trajectory point
        x = TolerancedNumber(2, 1, 3, samples=3)
        y = TolerancedNumber(2.5, 2, 3, samples=4)
        tp = TrajectoryPt([x, y, 0])
        
        # create robot and collision scene
        robot1 = Robot_2P3R([1, 1, 2, 1.5, 0.5])
        sc1 = [Rectangle(2, 1, 0.5, 0.5, 0)]
        
        # create the object
        Qv = [VirtualJointSolutions(robot1, tp, sc1),
              VirtualJointSolutions(robot1, tp, sc1)]
        
        ls = LazyGASolver(Qv)
        c = ls.create_population()
        

class TestGASOlver():
  def test_get_path(self):
    c = np.array([0, 1, 0])
    actual = solver.get_path(c)
    desired = np.array([[0, 0], [1, 0], [0, 2]])
    assert_almost_equal(actual, desired)
  
  def test_fitness(self):
    c = np.array([0, 1, 0])
    actual = solver.fitness(c)
    desired = -4.0
    assert_almost_equal(actual, desired)
  
  def test_create_chromosome(self):
    ch = solver.create_chromosome()
    
    # test path length
    actual1 = len(ch)
    desired1 = len(data)
    assert actual1 == desired1
    
    # test data type
    actual2 = [type(el) == np.int64 for el in ch]
    assert np.all(actual2)
    
    # test data range
    actual3 = [ch[i] in range(data_size[i]) for i in range(len(ch))]
    assert np.all(actual2)
  
  def test_create_population(self):
    # default size value
    actual1 = solver.create_population()
    s, n = actual1.shape
    assert s == 30
    assert n == len(data)
    # custom size value
    solver2 = GASolver(data, pop_size=15)
    actual1 = solver2.create_population()
    s, n = actual1.shape
    assert s == 15
    assert n == len(data)
  
  def test_mutate(self):
    # How do I test probabilistic functions?
    # There is a small change that the function works, but this test fails
    c = np.array([0, 1, 0])
    res = []
    for i in range(100):
      cm = solver.mutate(c, 0.8)
      res.append(np.all(c == cm))
    assert not np.all(res)
  
  def test_crossover(self):
    c1 = np.array([1, 2, 3, 4, 5])
    c2 = np.array([7, 8, 9, 10, 11])
    a1, a2 = solver.crossover(c1, c2, 1.0)
    assert not np.all(a1 == a2)
  
  def test_sort_population(self):
    pop = np.array([[0, 1, 0],
                    [0, 0, 0],
                    [1, 1, 1],
                    [1, 2, 1]])
    # f = [solver.fitness(c) for c in pop]
    # fitness of this pop: array([-4, -6, -5, -3])
    pop_sorted = np.array([[1, 2, 1],
                           [0, 1, 0],
                           [1, 1, 1],
                           [0, 0, 0]])
    actual = solver.sort_population(pop)
    assert_almost_equal(actual, pop_sorted)
  
  def test_new_generation(self):
    # for even populations the shape should be the same
    pop = np.array([[0, 1, 0],
                    [0, 0, 0],
                    [1, 1, 1],
                    [1, 2, 1]])
    actual1 = solver.new_generation(pop)
    assert_almost_equal(actual1.shape, pop.shape)
    # for add populations, size increases by one
    pop_odd = np.array([[0, 1, 0],
                        [0, 0, 0],
                        [1, 1, 1],
                        [1, 2, 1],
                        [1, 0, 1]])
    actual2 = solver.new_generation(pop_odd)
    a, _ = actual2.shape
    d, _ = pop_odd.shape
    assert a == d + 1
  
  def test_run(self):
    f_opt, p_opt, _ = solver.run()
    assert_almost_equal(f_opt, -3)
    # example data has two shortest paths with fitness -3
    assert np.all(p_opt == [1, 2, 0]) or np.all(p_opt == [1, 2, 1])

def test_get_shortest_path_ga():
  sol = get_shortest_path_ga(data)
  assert_almost_equal(sol['length'], 3)
  # example data has two shortest paths with fitness -3
  qs = sol['path']
  c1 = [1, 2, 0]; c2 = [1, 2, 1]
  d1 = np.vstack([data[i][c1[i]] for i in range(3)])
  d2 = np.vstack([data[i][c2[i]] for i in range(3)])
  assert np.allclose(qs, d1) or np.allclose(qs, d2)
