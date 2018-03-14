#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from multiprocessing import Pool, TimeoutError
import time
import os

from planning_cases import robot1, path1, scene1
from ppr.path import discretise

def check_pose(pose):
    res = []
    sol = robot1.ik(pose)
    for qsol in sol['q']:
        if not robot1.check_collision(qsol, scene1):
            res.append(qsol)
    return res

def check_traj_pt(poses):
    res = []
    for pi in poses:
        res.append(check_pose(pi))
    return res
    
def f(x):
    return x*x

if __name__ == '__main__':
    print(poses)
    start = time.time()
    res1 = []
    for tp in path1:
        poses = discretise(tp)
        res1.append(check_traj_pt(poses))
    for qs in res1: print(qs)
    end = time.time()
    print("For loop took {:.2f}s".format(end - start))
    
    
    start = time.time()
    tps = [discretise(tp) for tp in path1]
    with Pool(processes=4) as pool:
        result = pool.map(check_traj_pt, tps)
    end = time.time()
    print("Parallel took {:.2f}s".format(end - start))