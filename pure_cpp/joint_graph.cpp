// #include <boost/config.hpp>
#include <iostream>
#include <vector>
// #include <string>
#include <Eigen/Dense>
// #include <cmath> // for abs()
// #include <algorithm> // for reverse()
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>

// new type for robot joint values
typedef std::array<float, 6> joint_value;

int main() {
    using namespace std;
    using namespace Eigen;
    using namespace lemon;

    float test = -0.23558;
    cout << "abs(float): " << abs(test) << endl;

    // create random test data
    const int nrows = 3;
    const int ncols = 4;
    const int ndof = 6;
    MatrixXf data = MatrixXf::Random(nrows, ncols);
    //cout << "Matrix with random test data: \n";
    //cout << data << endl;

    ListDigraph g;

    // map containing graph data
    ListDigraph::NodeMap<joint_value> joint_values(g);
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
            joint_values[n] = { data(row, col), 0, 0, 0, 0, 0 };
            current.push_back(n);
            cout << g.id(n) << "(" << joint_values[n][0] << ")" << " ";
        }
        cout << "\n---------------\n";

        // connect with previous column if exists
        if (col == 0) {
            previous = current;
            current.clear();
        } else {
            for (auto nc : current) {
                for (auto np : previous) {
                    // add arc with calculated cost
                    a = g.addArc(np, nc);
                    c = abs(joint_values[np][0] - joint_values[nc][0]);
                    cost[a] = static_cast<int>(100 * c);
                    // print arc
                    cout << "(" << g.id(np) << ", " << g.id(nc) << ")  ";
                    cout << cost[a] << "\n";
                }
            }
            previous = current;
            current.clear();
        }
    }

    for (ListDigraph::NodeIt it(g); it != INVALID; ++it) {
        cout << g.id(it) << " | " << joint_values[it][0] << "\n";
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
    // cout << endl;
    
    cout << endl;
    return 0;
}