//=======================================================================
// Copyright 2001 Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee, 
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
#include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
// #include <boost/tuple/tuple.hpp>

int main() {
    //using namespace boost;
    
    // typedef property<edge_weight_t, float> EdgeProperty;
    // typedef adjacency_list<vecS, vecS, directedS, VertexProperty, EdgeProperty> Graph;

    // print bool as true or false instead of 1 or 0
    std::cout.setf(std::ios::boolalpha);

    typedef boost::property<boost::vertex_distance_t, float> VertexProperty;
    typedef boost::adjacency_list<boost::vecS,boost::vecS, boost::directedS, VertexProperty> Graph;

    Graph g;

    // manipulate vertices
    Graph::vertex_descriptor v1 = boost::add_vertex(g);
    Graph::vertex_descriptor v2 = boost::add_vertex(g);

    std::cout << v1 << " | " << v2 << std::endl;

    boost::add_vertex(g);
    boost::add_vertex(g);
    boost::add_vertex(g);

    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vs = boost::vertices(g);
    for (auto it=vs.first; it!=vs.second; ++it) {
        std::cout << *it << " | ";
    }
    std::cout << std::endl;

    // manipulate edges
    std::pair<Graph::edge_descriptor, bool> e1 = boost::add_edge(v1, v2, g);
    std::cout << e1.first << e1.second << std::endl;

    boost::add_edge(0, 2, g);
    boost::add_edge(1, 2, g);
    boost::add_edge(2, 2, g);

    std::pair<Graph::edge_iterator, Graph::edge_iterator> es = boost::edges(g);
    for (auto it=es.first; it!=es.second; ++it) {
        std::cout << *it << " | ";
    }
    std::cout << std::endl;

    std::cout << "Target of e1: " << boost::target(e1.first, g) << std::endl;

    // manipulate properties
    boost::property_map<Graph, boost::vertex_distance_t>::type distance = get(boost::vertex_distance_t(), g);
    boost::put(distance, 0, 3.7);
    boost::put(distance, 1, 1.9);

    std::cout << "Distance v1: " << boost::get(distance, v1) << std::endl;

    for (auto it=vs.first; it!=vs.second; ++it) {
        std::cout << boost::get(distance, *it) << " | ";
    }
    std::cout << std::endl;

}