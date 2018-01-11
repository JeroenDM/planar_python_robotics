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
using namespace boost;

typedef std::array<float, 6> joint_value;

float cost_function(joint_value, joint_value);

int main() {
    // create random test data
    const int nrows = 3;
    const int ncols = 4;
    const int ndof = 6;
    MatrixXf m = MatrixXf::Random(nrows, ncols);
    cout << m << endl;

    // setup graph
    struct joint_value_t {
        typedef vertex_property_tag kind;
    };

    //typedef boost::property<vertex_distance_t, float> VertexProperty;
    //typedef property<edge_weight_t, float> EdgeProperty;
    typedef boost::adjacency_list<vecS,vecS, directedS,
        boost::property<joint_value_t, joint_value>,
        boost::property<boost::edge_weight_t, int>> Graph;
    Graph g;
    property_map<Graph, joint_value_t>::type distance = get(joint_value_t(), g);
    //property_map<Graph, edge_weight_t>::type weight = get(edge_weight_t(), g);

    // create cost matrix
    // std::vector<Matrix<float, (nrows-1), ndof, RowMajor>> cms(ncols);
    // for (auto cm : cms) {
    //     for (int col=1, col<ncols; ++col) {
            
    //     }
    // }
    // for (int col=1; col<ncols; ++col) {
    //     cout << "---------------\n";
    //     cout << m.col(col) << "\n";
    //     for (int row=0; row < nrows; ++row) {
    //         m.col(col) = m.col(col).array() - 1.0;
    //     }
    // }

    // cout << "new Matrix\n";
    // cout << m << endl;

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
            joint_value j_temp = { m(row, col), 0, 0, 0, 0, 0 };
            put(distance, v_temp, j_temp);
            cur_vertices.push_back(v_temp);
        }

        // add edges from previous column to current column
        if (col == 0) {
            prev_vertices = cur_vertices;
            cur_vertices.clear();
        } else {
            for (auto it=prev_vertices.begin(); it!=prev_vertices.end(); ++it) {
                for (auto it2=cur_vertices.begin(); it2!=cur_vertices.end(); ++it2) {
                    // calculate cost
                    w_temp = cost_function(boost::get(distance, *it), boost::get(distance, *it2));
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

float cost_function(joint_value j1, joint_value j2) {
        float cost = 0;
        for (int i=0; i<j1.size(); ++i) {
            cost += abs(j1[i] - j2[i]);
        }
        return cost;
        // return abs(j1[0] - j2[0]);
}