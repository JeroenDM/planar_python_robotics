#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 10:23:28 2017

@author: jeroen
"""

import networkx as nx
import numpy as np


""" graph functions """

def create_graph(Q, CM):
    N = len(Q)
    Q_size = [len(q) for q in Q]
    G = nx.DiGraph()
    source_nodes = []
    target_nodes = []
    for i in range(N):
        for j in range(Q_size[i]):
            # add nodes
            node = str(i) + "|" + str(j)
            G.add_node(node)
            
            # remeber source nodes
            if i == 0:
                source_nodes.append(node)
            # remember target nodes
            if i == N-1:
                target_nodes.append(node)
            
            # add edges
            if i > 0:
                for k in range(Q_size[i-1]):
                    prev_node = str(i-1) + "|" + str(k)
                    G.add_edge(prev_node, node, weight = CM[i-1][k, j])
    return G, source_nodes, target_nodes

def shortest_path(G, source_nodes, target_nodes):
    f_opt = np.inf
    for n in source_nodes:
        f_vec = nx.single_source_dijkstra_path_length(G, source=n)
        f_vec = [f_vec[key] for key in target_nodes]
        f_min = min(f_vec)
        if f_min < f_opt:
            f_opt = f_min
            n_opt = target_nodes[f_vec.index(f_opt)]
            path = nx.dijkstra_path(G, source = n, target=n_opt)
    return f_opt, path

def get_shortest_path(Q):
    cost = create_cost_matrices(Q)
    graph, sn, tg = create_graph(Q, cost)
    del cost
    f_opt, path = shortest_path(graph, sn, tg)
    path = [int(s.split("|")[1]) for s in path]
    return f_opt, path

""" util functions """

def create_cost_matrices(Q):
    CM = []
    N = len(Q)
    Q_size = [len(q) for q in Q]
    for i in range(N-1):
        n1 = Q_size[i]
        n2 = Q_size[i+1]
        Mi = np.zeros((n1, n2))
        for j in range(n1):
            for k in range(n2):
                Mi[j, k] = np.sum(np.abs(Q[i][j] - Q[i+1][k]))
        CM.append(Mi)
    return CM

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from numpy.random import rand
    # create random testdata
    #np.random.seed(42)
    N_test = 8
    Q_size_test = [15]*N_test # change this if change N
    Q_test = [rand(Q_size_test[i], 3) for i in range(N_test)]
    
    cost = create_cost_matrices(Q_test)
    graph, sn, tg = create_graph(Q_test, cost)
    
    plt.figure()
    nx.draw_spectral(graph, with_labels=True)
    
    fs, path = shortest_path(graph, sn, tg)
    path = [int(s.split("|")[1]) for s in path]
    print(fs, path)



