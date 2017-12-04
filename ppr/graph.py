#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov 16 10:23:28 2017

@author: jeroen
"""

import networkx as nx
import numpy as np


""" graph functions """ 

# TODO double default of the and th_value
def create_graph(Q, CM, th=False, th_value=0.1):
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
                    cost_k = CM[i-1][k, j]
                    if th and cost_k > th_value:
                        pass # do not add edge
                    else:
                        prev_node = str(i-1) + "|" + str(k)
                        G.add_edge(prev_node, node, weight = cost_k)
    return G, source_nodes, target_nodes

def shortest_path(G, source_nodes, target_nodes):
    f_opt = np.inf
    path = None
    for n in source_nodes:
        f_vec = nx.single_source_dijkstra_path_length(G, source=n)
        try:
            f_vec = [f_vec[key] for key in target_nodes]
            f_min = min(f_vec)
            if f_min < f_opt:
                f_opt = f_min
                n_opt = target_nodes[f_vec.index(f_opt)]
                path = nx.dijkstra_path(G, source = n, target=n_opt)
        except KeyError:
            print("[ppr.graph.py] No path found for this source node " + n)
    return f_opt, path

def shortest_path2(G, source_nodes, target_nodes):
    l = np.inf
    for i, si in enumerate(source_nodes):
        print("source node " + str(i))
        for j, tj in enumerate(target_nodes):
            li = nx.dijkstra_path_length(G, si, tj)
            if li < l:
                l = li
                im = i
                jm = j
    return nx.dijkstra_path(G, source_nodes[im], target_nodes[jm])

def get_shortest_path(Q):
    cost = create_cost_matrices(Q)
    graph, sn, tg = create_graph(Q, cost)
    del cost
    f_opt, path = shortest_path(graph, sn, tg)
    if path == None:
        print("[ppr.graph.py] no path found in TOTAL!")
        return False, None
    else:
        path = [int(s.split("|")[1]) for s in path]
        path = [Q[i][path[i]] for i in range(len(Q))]
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

def create_single_joint_graph(Q, dist_th=0.5):
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
                    dist = np.abs(Q[i][j] - Q[i-1][k])
                    if dist > dist_th:
                        pass # do not add edge
                    else:
                        prev_node = str(i-1) + "|" + str(k)
                        G.add_edge(prev_node, node, weight = dist)
    return G, source_nodes, target_nodes

if __name__ == "__main__":
    print("-----test graph.py-----")
    print("-----------------------")
    import matplotlib.pyplot as plt
    from numpy.random import rand
    # create random testdata
    np.random.seed(42)
    N_test = 6
    Q_size_test = [15]*N_test # change this if change N
    Q_test = [rand(Q_size_test[i], 3) for i in range(N_test)]
    
#    cost = create_cost_matrices(Q_test)
#    graph, sn, tg = create_graph(Q_test, cost)
#    
##    plt.figure()
##    nx.draw_spectral(graph, with_labels=True)
#    
#    fs, path = shortest_path(graph, sn, tg)
#    if fs:
#        path = [int(s.split("|")[1]) for s in path]
#        print(fs, path)
    
    print("test single joint graph")
    print("-----------------------")
    Q1 = [q[:, 0] for q in Q_test]
    
    
    
#    c1 = create_cost_matrices(Q1)
#    g1, sn1, tn1 = create_graph(Q1, c1)
#    fs1, path1 = shortest_path(g1, sn1, tn1)
#    print(path1)
    
    f1, p1 = get_shortest_path(Q1)
    
    if f1:
        print(p1)
        plt.figure()
        for i, q1 in enumerate(Q1):
            ind = np.ones(q1.shape) * i
            plt.plot(ind, q1, 'g*')
            plt.plot(i, p1[i], 'r*')
    else:
        print("No path found!")
    
    plt.figure()
    g1, s1, t1 = create_single_joint_graph(Q1)
    nx.draw_spectral(g1, with_labels=True)


