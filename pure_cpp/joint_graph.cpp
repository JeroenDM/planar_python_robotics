// #include <boost/config.hpp>
#include <iostream>
#include <vector>
// #include <string>
#include <Eigen/Dense>
// #include <cmath> // for abs()
// #include <algorithm> // for reverse()
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

using namespace std;
using namespace Eigen;
using namespace lemon;

typedef array<float, 6> joint_value;

float cost_function(joint_value, joint_value);

int main() {
    // create random test data
    const int nrows = 3;
    const int ncols = 4;
    const int ndof = 6;
    MatrixXf m = MatrixXf::Random(nrows, ncols);
    cout << "Matrix with random test data: \n";
    cout << m << endl;

    // defualt lemon graph
    ListDigraph g;

    // map containing graph data
    ListDigraph::NodeMap<joint_value> joint_value(g);
    ListDigraph::ArcMap<int> cost(g);

    // add data from matrix m to graph
    ListDigraph::Node n;
    ListDigraph::Arc a;
    float c; // temp cost variable
    vector<ListDigraph::Node> current;
    vector<ListDigraph::Node> previous;
    int row = 0;
    for (int col = 0; col < ncols; ++col) {
        // add joint values  to graph
        for (int row = 0; row < nrows; ++row) {
            n = g.addNode();
            joint_value[n] = { m(row, col), 0, 0, 0, 0, 0 };
            current.push_back(n);
        }

        // connect with previous column if exists
        if (col == 0) {
            previous = current;
            current.clear();
        } else {
            for (auto nc : current) {
                for (auto np : previous) {
                    a = g.addArc(np, nc);
                    c = cost_function(joint_value[np], joint_value[nc]);
                    cost[a] = static_cast<int>(100 * c);
                    cout << "(" << g.id(np) << ", " << g.id(nc) << ")  ";
                    cout << cost[a] << "\n";
                }
            }
            previous = current;
            current.clear();
        }
    }

    for (ListDigraph::NodeIt it(g); it != INVALID; ++it) {
        cout << g.id(it) << " | " << joint_value[it][0] << "\n";
    }
    cout << endl;

    // calculate shortest path
    ListDigraph::Node start = g.nodeFromId(0);
    ListDigraph::Node goal = g.nodeFromId(10);

    Path<ListDigraph> p;
    ListDigraph::NodeMap<int> dist(g);
    Dijkstra<ListDigraph> dk(g, cost);
    dk.distMap(dist);
    dk.init();
    dk.run(start, goal);

    // minimum distance to the last node
    cout << "Shortest path length: \n";
    cout << dist[goal] << endl;

    // go from last node to start and put the nodes in a vector
    vector<ListDigraph::Node> path;
    for (ListDigraph::Node n = goal; n != start; n = dk.predNode(n)) {
        path.push_back(n);
    }
    path.push_back(start);

    // reverse path vector
    reverse(path.begin(), path.end());

    // the final path
    cout << "The path: \n";
    for (auto p : path) {
        cout << g.id(p) << " ";
    }
    cout << endl;
    
    return 0;
}

float cost_function(joint_value j1, joint_value j2) {
        float cost = 0;
        for (int i=0; i<j1.size(); ++i) {
            cost += abs(j1[i] - j2[i]);
        }
        return cost;
        // return abs(j1[0] - j2[0]);
}