#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Find global optimal path using a genetic algorithm
"""

import numpy as np
from ppr.path import TolerancedNumber, TrajectoryPt

def cp(array):
  return np.cumprod(array)[-1]

def setup_ranges(tp):
  ranges = []
  for i in range(tp.dim):
    if tp.hasTolerance[i]:
      ranges.append(tp.p[i].range)
    else:
      ranges.append(np.array([tp.p[i]]))
  return ranges

class VirtualArray():
  """ Virtual version of an N D array to store big grids.
  
  By supplying the discretised individual axes of an N-dimensional space
  this class can calculate grid points on the fly, instead of calculating
  and storing the whole N-dimensional array at once.
  
  In essence this converts an single integer number from the range
  [0, n1*n2*n3-1] into a list of values from the range arrays.
  [range1[n1], range[1][n2], ...] in a deterministic way.
  
  Attributs
  ---------
  ranges : list of numpy.ndarray
  sizes : list of int
    The size of the different discrete dimensions.
  total_size : int
    The total number of grid points that can be accesed.
  """
  def __init__(self, ranges):
    """ Create virtual array
    
    Paramters
    ---------
    ranges : list of numpy.ndarray
      Every element in the list represents a discretised axis in and
      n-dimensional space, where n is the list length.
    """
    self.ranges = ranges
    self.sizes = [len(a) for a in self.ranges]
    self.total_size  = cp(self.sizes)
  
  def __getitem__(self, index):
    """ Implement index slicing
    
    The method get's the syntac object[i] working.
    Therefore the object acts as a 1D numpy array.
    """
    si = self.sample(index, self.sizes)
    return [self.ranges[k][si[k]] for k in range(3)]

  def sample(self, I, l):
    a = [0] * len(l)
    l = np.array(l)
    
    # initial case i = 0
    D, R = divmod(I, l[0])
    a[0] = int(R)
    I   -= R
    for i in range(1, len(l) - 1):
        D, R = divmod(I, cp(l[0:i+1]))
        a[i] = int(R / cp(l[0:i]))
        I -= R
    # last element case
    a[-1] = int(D)
    
    return a

x = TolerancedNumber(2, 1, 3, samples=3)
y = TolerancedNumber(0, 0, 1, samples=4)
tp = TrajectoryPt([x, y, 9.9])

s = VirtualArray(setup_ranges(tp))
print(s.total_size)

P1 = s.total_size
for i in range(P1):
  print(s[i])