#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 14 20:10:06 2017

@author: jeroent

test problem
Q1 Q2 Q3 are nx3 matrices representing n nodes in a graph
the cost to go from Q1 to Q2 is the squared difference of the two 1x3 vectors
find the shortest path from Q1 to Q3

do until satisfied
    do until new population
        select two parents with probability
        crossover with rate
    batch mutate new populations
convetions
return np.ndarray if 2D or 1D
return list with 2D arrays if higher dimension
"""
import numpy as np
from numpy.random import rand, randint, choice

#=============================================================================
# Main functions
#=============================================================================


def get_shortest_path(Q, method='ga'):
    if method == 'ga':
        return _get_shortest_path_ga(Q)
    else:
        raise ValueError("Method not implemented: " + method)

def _get_shortest_path_ga(Q,
                          cross_rate=0.9, mut_rate=0.2,
                          pop_init=30, iters=200):
    # set ga parameters based on test data
    gp['Q'] = Q
    gp['N'] = len(Q)
    gp['Q_size'] = [len(q) for q in Q]
    
    # Check whether at least one path exists
    if 0 in gp['Q_size']:
        return False, None
    
    # set crossover and mutation rate
    gp['cross_rate'] = cross_rate
    gp['mut_rate'] = mut_rate
    
    # create initial population and run ga algorithm
    pop = create_population(pop_init)
    f_opt, path_index, fvec = run_ga(pop, iters=iters)
    path = [ Q[i][path_index[i], :] for i in range(gp['N']) ]
    return f_opt, path

#=============================================================================
# GA parameters defaults
#=============================================================================
gp = {}
gp['N'] = 1 # path size
gp['Q_size'] = [1] # chromosome length for every path point
gp['Q'] = [rand(gp['Q_size'][i], 3) for i in range(gp['N'])] # random default data
gp['cross_rate'] = 0.8
gp['mut_rate'] = 0.1



#=============================================================================
# GA functions
#=============================================================================
def run_ga(init_pop, iters=100):
    cnt = iters
    pop = init_pop
    f_opt = np.inf
    f_all = []
    while(cnt > 0):
        pop = sort_population(pop)
        
        f_cur = fitness(pop[0])
        f_all.append(f_cur)
        if f_cur < f_opt:
            f_opt = f_cur
        
        pop = new_generation(pop)
        cnt -= 1
    f_all = np.array(f_all)
    return f_opt, pop[0], f_all

def create_chromosome():
    return np.array([randint(0, gp['Q_size'][i]) for i in range(gp['N'])])

def create_population(size=30):
    pop = [create_chromosome() for i in range(size)]
    return np.array(pop)

def sort_population(pop):
    fit = [fitness(ci) for ci in pop]
    fit = np.array(fit)
    return pop[ fit.argsort() ]

def fitness(c):
    path = get_path(c)
    dp   = np.diff(path, axis=0)
    return np.sum(np.abs(dp))

def mutate(c, rate):
    """TODO fix low mutation rate
    if the random replacement gene is the same
    as the original
    """
    if rand() < rate:
        c_m = c.copy()
        m_i = randint(0, len(c))
        c_m[m_i] = randint(0, gp['Q_size'][m_i])
        return c_m
    else:
        return c

def crossover(par1, par2, rate):
    # single point crossover
    if rand() < rate:
        index = randint(0, len(par1))
        c1 = np.hstack(( par1[:index], par2[index:] ))
        c2 = np.hstack(( par2[:index], par1[index:] ))
    else:
        c1 = par1
        c2 = par2
    return c1, c2

def new_generation(pop):
    p_size = len(pop) # if an odd number, the next pop will have size+1
    current_size = 0
    new_pop = []
    while(current_size < p_size):
        # select couple
        # better choose first elements than last
        prob = np.linspace(1, 0, p_size)
        prob = prob / np.sum(prob)
        prob = list(prob)
        couple = choice(p_size, 2, p = prob)
        # mate couple
        c1, c2 = crossover(pop[couple[0]], pop[couple[1]], gp['cross_rate'])
        # mutate children
        c1 = mutate(c1, gp['mut_rate'])
        c2 = mutate(c2, gp['mut_rate'])
        # add children to new population
        new_pop.append(c1)
        new_pop.append(c2)
        current_size += 2
    return np.array(new_pop)
        
#=============================================================================
# UTIL functions
#=============================================================================
def get_path(c):
    return np.array([gp['Q'][i][c[i]] for i in range(gp['N'])])

def brute_force(iters = 100):
    f_all = np.zeros(iters)
    f_opt = np.inf
    for i in range(iters):
        ci =  create_chromosome()
        f_all[i] = fitness(ci)
        if f_all[i] < f_opt:
            f_opt = f_all[i]
    
    return f_opt, f_all

#=============================================================================
# Testing
#=============================================================================
if __name__ == "__main__":
    gp['N'] = 4
    gp['Q_size'] = [8]*gp['N']
    gp['Q'] = [rand(gp['Q_size'][i], 3) for i in range(gp['N'])]
    
    print("---------------")
    print("test chromosome")
    print("---------------")
    c1 = create_chromosome()
    print(c1)
    
    print("----------------------")
    print("test path and fitness")
    print("----------------------")
    print(get_path(c1))
    f1 = fitness(c1)
    print("fitness of this path: " + str(f1))
    
    print("---------------")
    print("test population")
    print("---------------")
    p1 = create_population(5)
    print(p1)
    
    print("----------------------")
    print("mutate the previous path with rate 0.9")
    print("----------------------")
    p2 = []
    for ci in p1:
        p2.append(mutate(ci, 0.9))
    p2 = np.array(p2)
    print(p2)
    
    print("----------------------")
    print("crossover genes from the population")
    print("----------------------")
    for i in range( len(p1) - 1 ):
        print("crossing", p1[i], p1[i+1])
        c1, c2 = crossover(p1[i], p1[i+1], 0.9)
        print("------->", c1, c2)
        print("")
    
    print("----------------------")
    print("create new population from prev one")
    print("----------------------")
    print("Old pop")
    print(p1)
    print("new pop")
    p2 = new_generation(p1)
    print(p2)
    
    print("----------------------")
    print("sort populaiont")
    print("----------------------")
    fp1 = [fitness(c) for c in p1]
    print(fp1)
    p3 = sort_population(p1)
    print(p3)
    fp3 = [fitness(c) for c in p3]
    print(fp3)
    
    print("----------------------")
    print("TEST GA vs brute force")
    print("----------------------")
    p4 = create_population(10)
    
    fsol, fpop, fvec = run_ga(p4, iters=100)
    fsol2, fvec2 = brute_force(iters = 1000)
    
    print("path " + str(fpop))
    
    ivec = np.linspace(0, 100*10, 100)
    ivec2 = np.linspace(0, 100*10, 1000)
    import matplotlib.pyplot as plt
    plt.figure()
    plt.plot(ivec, fvec, '.', ivec2, np.minimum.accumulate(fvec2), '.')
    plt.show()
    
    print("----------------------")
    print("get shortest path")
    print("----------------------")
    fopt, p = get_shortest_path(gp['Q'])
    print("path " + str(p))

