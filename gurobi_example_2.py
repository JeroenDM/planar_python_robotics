#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jan 18 14:05:51 2018

@author: jeroen
"""

import gurobipy as gu


import numpy as np

q1 = [0.0, 0.0, 0.0]
q2 = [5.0, 1.0, 0.5]
q3 = [8.0, 1.1, 3.0]
Q = [q1, q2, q3]

print(Q)

def cost(val1, val2):
    return abs((val1-val2))

m = gu.Model("simple1")

i1 = range(0, len(q1))
j1 = m.addVars(i1, vtype=gu.GRB.BINARY, name="j1")
i2 = range(0, len(q2))
j2 = m.addVars(i1, vtype=gu.GRB.BINARY, name="j2")
i3 = range(0, len(q3))
j3 = m.addVars(i1, vtype=gu.GRB.BINARY, name="j3")

obj = gu.QuadExpr()

for i in i1:
    for k in i2:
        obj += cost(q1[i], q2[k]) * j1[i] * j2[k]

for i in i2:
    for k in i3:
        obj += cost(q2[i], q3[k]) * j2[i] * j3[k]

m.setObjective(obj, gu.GRB.MINIMIZE)

for ji, ii in zip([j1, j2, j3], [i1, i2, i3]):
    m.addConstr(sum(ji[ind] for ind in ii) == 1)

#for i in i1:
#    for j in i2:
#        c = cost(j1[i] * q1[i], j2[j] * q2[j])

#m.setObjective(cost(Q[0, x], Q[1, y]) + cost(Q[0, y], Q[1, z]))
#m.setObjective(sum(j1[i] for i in i1), gu.GRB.MAXIMIZE )
#
##m.addConstr(x + y >= 3)
#
m.optimize()
#
for v in m.getVars():
    print(v.varName, v.x)