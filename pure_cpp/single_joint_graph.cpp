// #include <boost/config.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <Eigen/Dense>
#include <cmath> // for abs()
#include <algorithm>

using namespace std;
using namespace Eigen;
using namespace boost;

int main() {
    // create random test data
    const int nrows = 3;
    const int ncols = 4;
    MatrixXd m = MatrixXd::Random(nrows, ncols);
    cout << m << endl;

    // setup graph
    //typedef boost::property<vertex_distance_t, float> VertexProperty;
    //typedef property<edge_weight_t, float> EdgeProperty;
    typedef boost::adjacency_list<vecS,vecS, directedS,
        boost::property<vertex_distance_t, float>,
        boost::property<boost::edge_weight_t, int>> Graph;
    Graph g;
    property_map<Graph, vertex_distance_t>::type distance = get(vertex_distance_t(), g);
    //property_map<Graph, edge_weight_t>::type weight = get(edge_weight_t(), g);

    // add data to graph
    float max_distance = 0.8;
    vector<Graph::vertex_descriptor> prev_vertices;
    vector<Graph::vertex_descriptor> cur_vertices;
    Graph::vertex_descriptor v_temp;
    pair<Graph::edge_descriptor, bool> e_temp;
    float w_temp;
    for (int col=0; col<ncols; ++col) {
        // create vertices for current column
        cout << "\nColumn: " << col << "\n";
        for (int row=0; row<nrows; ++row) {
            v_temp = add_vertex(g);
            put(distance, v_temp, m(row, col));
            cur_vertices.push_back(v_temp);
        }
        // add edges from previous column to current column
        if (col == 0) {
            prev_vertices = cur_vertices;
            cur_vertices.clear();
        } else {
            for (auto it=prev_vertices.begin(); it!=prev_vertices.end(); ++it) {
                for (auto it2=cur_vertices.begin(); it2!=cur_vertices.end(); ++it2) {
                    w_temp = abs(boost::get(distance, *it) - boost::get(distance, *it2));
                    if (w_temp <= max_distance) {
                        e_temp = add_edge(*it, *it2, w_temp, g);
                        cout << e_temp.first << " | " << w_temp << "\n";
                        //boost::put(weight, e_temp.first, dist_temp);
                        //cout << e_temp.first << " | " << boost::get(weight, e_temp.first) << "\n";
                    }
                }
            }
            prev_vertices.clear();
            prev_vertices = cur_vertices;
            cur_vertices.clear();
        }
    }
    cout << endl;

    // array to store the predecessor of each node in the shortest path tree
    std::array<int, nrows*ncols> v_array;

    boost::dijkstra_shortest_paths(g, 0,
        boost::predecessor_map(v_array.begin()));
    
    int u = 0;
    for (auto i : v_array) {
        cout << u << ":" << i << " | ";
        ++u;
    }
    cout << endl;

    // extract shortest path
    std::vector<int> path;
    int goal = 11;
    int current_point = goal;
    int next_point;
    bool start_reached = false;
    path.push_back(goal);
    while (!start_reached) {
        next_point = v_array[current_point];
        path.push_back(next_point);
        if (next_point == current_point) {
            cout << "\nThis goal cannot be reached\n" << endl;
            start_reached = true;
        } else if (next_point == 0) {
            cout << "\nShortest path found!\n";
            start_reached = true;
        } else {
            current_point = next_point;
        }
    }

    reverse(path.begin(), path.end());
    for (auto p : path) cout << p << " ";
    cout << endl;
}