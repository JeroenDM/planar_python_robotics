// #include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <Eigen/Dense>
#include <cmath> // for abs()
#include <algorithm> // for reverse()
#include <vector>

using namespace std;
using namespace Eigen;
//using namespace boost;

float cost_function(std::array<float, 6>, std::array<float, 6>);

int main() {
    // create random test data
    const int nrows = 3;
    const int ncols = 4;
    const int ndof = 6;
    MatrixXf m = MatrixXf::Random(nrows, ncols);
    cout << "The random cost matrix: \n";
    cout << m << endl;

    // graph vertice and edge structure
    struct JointPt {
        std::array<float, 6> joint_value;
    };

    struct Edge {
        float cost;
    };

    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, JointPt, Edge> Graph;
    Graph g;

    // add 5 vertives
    for (int i = 0; i<5; ++i) {
        boost::add_vertex(g);
    }

    // iterate over vertices
    std::pair<Graph::vertex_iterator, Graph::vertex_iterator> vs = boost::vertices(g);
    for (auto it=vs.first; it != vs.second; ++it) {
        cout << *it;
    }
    cout << endl;

    // add edges
    Graph::edge_descriptor e1, e2, e3, e4;
    e1 = boost::add_edge(0, 1, g).first;
    e2 = boost::add_edge(1, 4, g).first;
    e3 = boost::add_edge(0, 3, g).first;
    e4 = boost::add_edge(3, 4, g).first;

    g[e1].cost = 4;
    g[e2].cost = 8;
    g[e3].cost = 5;
    g[e4].cost = 2;

    // iterate over edges

    //cout << "--- The graph ---\n";
    //cout << e1.first << endl;
    std::pair<Graph::edge_iterator, Graph::edge_iterator> es = boost::edges(g);
    for (auto it=es.first; it != es.second; ++it) {
        cout << *it << " has cost: " << g[*it].cost << "\n";
    }

    // calculate shortest path
    array<int, nrows*ncols> v_array; // fixed length 1D array

    //boost::dijkstra_shortest_paths(g, 0,
    //    boost::predecessor_map(v_array.begin()));
}

float cost_function(std::array<float, 6> j1, std::array<float, 6> j2) {
        float cost = 0;
        for (int i=0; i<j1.size(); ++i) {
            cost += abs(j1[i] - j2[i]);
        }
        return cost;
        // return abs(j1[0] - j2[0]);
}