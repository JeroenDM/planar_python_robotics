#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 18 09:16:40 2018

@author: jeroen
"""
import numpy as np
from queue import Queue

def cost_function(node1, node2):
    return np.sum(np.abs(node1.data - node2.data))

class Node:
    def __init__(self, path_index, sample_index, data):
        self.path_index = path_index
        self.sample_index = sample_index
        self.data = data
        self.dist = np.inf
        self.parent = None
        self.visited = False
    
    def __str__(self):
        s = "Node (" + str(self.path_index) + ", "
        s += str(self.sample_index) + ") data: " + str(self.data)
        return s

class Graph:
    def __init__(self, data):
        self.data = data
        self.node_array = self.data_to_node_array()
        self.path_length = len(data)
    
    def data_to_node_array(self):
        na = []
        for i, data_col in enumerate(self.data):
            na.append([])
            for j, data_row in enumerate(data_col):
                na[-1].append(Node(i, j, data_row))
        return na
    
    def reset(self):
        for col in self.node_array:
            for node in col:
                node.dist = np.inf
                node.parent = None
                node.visited = False
    
    def get_neighbours(self, node):
        next_path_index = node.path_index + 1
        if next_path_index >= self.path_length:
            return []
        else:
            return self.node_array[next_path_index]
        
    def get_reachable_neighbours(self, node, max_cost = 16):
        nb = self.get_neighbours(node)
        rnb = []
        for next_node in nb:
            if cost_function(node, next_node) < max_cost:
                rnb.append(next_node)
        return rnb
    
    def get_path(self, target_path_index=None):
        # set default target path index if not specified
        if target_path_index is None:
            target_path_index = self.path_length-1
        
        # look for the closest node in the target columns
        min_dist = np.inf
        closest_node = None
        for node in self.node_array[target_path_index]:
            if node.dist < min_dist:
                min_dist = node.dist
                closest_node = node
        
        if closest_node is None:
            return [-1], np.inf
        else:
            node_sample_index_list = []
            current_node = closest_node
            while(current_node.parent is not None):
                node_sample_index_list.append(current_node.sample_index)
                current_node = current_node.parent
            
            node_sample_index_list.reverse()
            return node_sample_index_list, min_dist
    
    def find_partial_path(self):
        # start at the back and look where the graph search got
        # before getting out of reachable neighbours
        current_path_index = self.path_length - 1
        found_shorter_path = False
        while (current_path_index > 0 and not found_shorter_path):
            found_shorter_path = np.any([n.parent is not None for n in self.node_array[current_path_index]])
            current_path_index -= 1
        
        if found_shorter_path:
            return current_path_index + 1
        else:
            return None
            
    
    def run_multi_source_bfs(self, max_cost = 16, start_path_index=0):
        self.reset()
        Q = Queue()
        # add dummy before the first column
        # or the column specified by start_path_index
        # with distance zero to all these nodes
        dummy_node = Node(-1, 0, 0)
        for node in self.node_array[start_path_index]:
            node.parent = dummy_node
            node.visited = True
            node.dist = 0
            Q.put(node)
        
        # the dummy note is not in the queue
        # assume it is already handles and got us in a state
        # where all distances to the nodes in the first columns are 0
        # therefore these nodes are added to the queue in the above loop
        
        # now continue running the algorithm as usual
        while(not Q.empty()):
            current_node = Q.get()
            nb = self.get_reachable_neighbours(current_node, max_cost=max_cost)
            # nb is an empty list if their are no neighbours
            # TODO double cost function calculation
            
            for node in nb:
                new_dist = current_node.dist + cost_function(current_node, node)
                if new_dist < node.dist:
                    node.dist = new_dist
                    node.parent = current_node
                
                if not node.visited:
                    Q.put(node)
                    node.visited = True
                
def get_shortest_path(data):
    g = Graph(data)
    g.run_multi_source_bfs()
    pi, cost = g.get_path()
    if pi[0] == -1:
        return {'success': False}
    else:
        path = path_index_to_path(pi, data)
        return {'success': True, 'path': path, 'length': cost}


def path_index_to_path(pi, data):
    res = []
    for i in range(len(pi)):
        qki = data[i][pi[i]]
        res.append(qki)
    return res

def get_shortest_path2(data, max_step_cost=5):
    g = Graph(data)
    g.run_multi_source_bfs(max_cost = max_step_cost)
    pi, cost = g.get_path()
    if pi[0] == -1:
        split_index = g.find_partial_path()
        if split_index is None:
            print("got stuck after first trajectory point, look after this point")
            return {'success': False}
        
        # find the first part up to the split index
        pi1, cost1 = g.get_path(target_path_index=split_index)
        path1 = path_index_to_path(pi1, data)
        
        # look for a second part of the path starting from split index
        g.run_multi_source_bfs(max_cost = max_step_cost,
                             start_path_index=split_index+1)
        pi2, cost2 = g.get_path()
        path2 = path_index_to_path(pi2, data[split_index+1:])
        return {'success': False, 'path': path1, 'length': cost1,
                'i': split_index, 'path2': path2}
    else:
        path = path_index_to_path(pi, data)
        return {'success': True, 'path': path, 'length': cost}

def get_shortest_path3(data, max_step_cost=5):   
    # iteratively find shortest path in subset of data, start with ass data
    current_data = data
    current_start_index = 0
    finished = False
    paths = []
    path_start_indices = []
    path_costs = []
    while(not finished and current_start_index < (len(data)-2)):
        res = get_shortest_path2(current_data)
        paths.append(res['path'])
        path_start_indices.append(current_start_index)
        path_costs.append(res['length'])
        
        if res['success']:
            finished = True
        else:
            current_start_index = res['i'] + 1
            current_data = data[current_start_index:]
    
    if finished:
        return {'success': True, 'paths': paths,
                'split_points': path_start_indices,
                'costs': path_costs}
    else:
        return {'success': False, 'paths': paths,
                'split_points': path_start_indices,
                'costs': path_costs}

    
#data1 = [np.array([[0, 0]]),
#         np.array([[1, -1], [1, 0], [1, 1]]),
#         np.array([[0, 2], [2, 2]]),
#         np.array([[4, 5], [5, 9]]),
#         np.array([[4, 6], [5, 10]]),
#         np.array([[4, 13], [5, 11]])]
#
#res = get_shortest_path3(data1)
#print(res)

#g = Graph(data1)
#g.run_multi_source_bfs(max_cost = 5)
#res = g.get_path()
#
#ind = g.find_partial_path()
#res1 = g.get_path(target_path_index=ind)
#
#g.run_multi_source_bfs(max_cost=5, start_path_index=ind+1)
#res2 = g.get_path()
#
#print(res)
#print(res1)
#print(res2)

#    def run_breath_first_search(self, start_node):
#        Q = Queue()
#        Q.put(start_node)
#        
#        start_node.visited = True
#        start_node.dist = 0
#        
#        while(not Q.empty()):
#            current_node = Q.get()
#            nb = self.get_neighbours(current_node)
#            
#            if nb is None:
#                print("End of the graph reached")
#            else:
#                for node in nb:
#                    new_dist = current_node.dist + cost_function(current_node, node)
#                    if new_dist < node.dist:
#                        node.dist = new_dist
#                        node.parent = current_node
#                    
#                    if not node.visited:
#                        Q.put(node)
#                        node.visited = True