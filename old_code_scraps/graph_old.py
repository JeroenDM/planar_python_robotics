#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 15:58:00 2017

@author: jeroen
"""

import numpy as np

class Edge:
    def __init__(self, id_int, from_id, to_id, properties = {}):
        """ create new edge
        
        Parameters
        ----------
        properties: dict
            A dictionary with edge properties that can be freely choosen
        
        Returns
        -------
        int
            the unique id of this edge
        """
        self.id = id_int
        self.fid = from_id
        self.tid = to_id
        self.prop = properties

class Node:
    def __init__(self, parent):
        self.id = id(self)
        if parent:
            self.pid = parent.id
        else:
            self.pid = None # has no parent
        self.childs = []
    
    def add_child(self, child):
        self.childs.append(child)
    
    def __str__(self):
        num_c = len(self.childs)
        msg = "A node with " + str( num_c ) + " childs.\n"
#        if num_c > 0:
#            msg += "In turn having:\n--------------------\n"
#            for c in self.childs:
#                msg += str(c) + "\n"
        return msg

#class Graph:
#    """ Implementation of a directed graph """
#    def __init__(self):
#        self.vertice_counter = 0
#        self.path_id_counter = 0
#        self.vertices = [] # ordered by path id in 2d list ?
#        self.edge_counter = 0
#        self.edges = []
#        self.ll = [] # link list for fast lookup
#
#    def add_vertice(self, path_id):
#        new_vert = Vertice(self.vertice_counter, path_id)
#        
#        if 
#        self.vertices.append(new_vert)
#        self.vertice_counter += 1
#        self.ll.append([])
#        return new_vert
#    
#    def add_edge(self, vert_from, vert_to, **arg):
#        new_edge = Edge(self.edge_counter, vert_from.id, vert_to.id, **arg)
#        self.edges.append(new_edge)
#        self.edge_counter += 1
#        self.ll[vert_from.id].append(vert_to.id)
#        return new_edge


if __name__ == "__main__":
    
    main_node = Node(None) # -1 meaning no parent
    
    # add child for every traj point
    for i in range(4):
        c = Node(main_node)
        main_node.add_child(c)
        
        # add random joint points
        nj = np.random.randint(1, 3)
        for j in range(nj):
            cc = Node(c)
            c.add_child(cc)
    
    print(main_node)
    
    
    
    
    