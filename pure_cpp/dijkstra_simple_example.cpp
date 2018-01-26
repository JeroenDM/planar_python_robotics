#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>      // std::stringstream (also in Eigen?)
// #include <Eigen/Dense>

#include "dijkstra.hpp"

void read_file(std::string filename, graph::graph_data& gd);

//===========================================
// MAIN
//===========================================

int main() {
    using namespace std;
    using namespace graph;
    //using namespace Eigen;
    
    graph_data data;
    read_file("simple_example.txt", data);
    //print_test_data(data);

    Graph g;
    add_data_to_graph(data, g);

    run_dijkstra(&g[0][0], g);

    cout << "Shortest path" << endl;
    // find last node with shortest distance to start
    float min_dist = INF;
    Node* goal;
    for (auto n : g[2]) {
        if (n.dist < min_dist) {
            goal = &n;
            min_dist = n.dist;
        }
    }
    vector<Node*> p = get_path(goal);
    print_nodes(p);

    return 0;
}

//===========================================
// functions
//===========================================

void read_file(std::string filename, graph::graph_data& gd) {
    // open file
    std::ifstream infile(filename);

    // check if opened succesfull
    if (!infile) std::cout << "Failed to open file." << std::endl;

    // read line by line and save in temporary string "line"
    std::string line;
    int i = 0;
    while(infile) {
        std::string str_input;
        getline(infile, str_input);
        if (str_input != ""){
            std::stringstream line_data(str_input);
            std::string temp;
            std::vector<float> new_column;
            gd.push_back(new_column);
            while (getline(line_data, temp, ',')) {
                gd.back().push_back(stof(temp));
            }
        }
    // file is closed when infile goes out of scope
    }
}