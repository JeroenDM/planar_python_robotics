#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <cmath>

typedef std::vector<std::vector<float>> graph_data;

// util functions
void read_file(std::string filename, graph_data& gd);
void print_test_data(graph_data& gd);

// graph functions
struct Node {
    int path_index;
    int sample_index;
    float* value;
    Node* parent;
};

void print_nodes(std::vector<Node>);

typedef std::vector<std::vector<Node>> Graph;

void add_data_to_graph(graph_data& gd, Graph& g);

float cost_function(Node, Node);
std::vector<Node> get_neighbors(Node, Graph& g);

int main() {
    using namespace std;
    using namespace Eigen;

    // read random test data from text file
    vector<float> test;
    
    graph_data data;
    read_file("data.txt", data);
    //print_test_data(data);

    // test Node structure
    Node node1 = {0, 0, &data[0][0]};
    Node node2 = {1, 0, &data[1][0]};
    vector<Node> ns = {node1, node2};
    print_nodes(ns);
    cout << cost_function(node1, node2) << endl;

    Graph g;
    add_data_to_graph(data, g);
    for (auto col : g) {
        cout << "--------\n";
        print_nodes(col);
    }
    cout << "--------" << endl;

    vector<Node> nb;
    nb = get_neighbors(g[0][0], g);
    print_nodes(nb);

    // for (int i = 0; i < data.size(); ++i) {
    //     cout << "-------------\n";
    //     vector<Node> new_column;
    //     g.push_back(new_column);
    //     for (int j = 0; j < data[i].size(); ++j) {
    //         Node new_node = {i, j, &data[i][j]};
    //         g[i].push_back(new_node);
    //     }
    //     print_nodes(g[i]);
    // }
    
    return 0;
}

void add_data_to_graph(graph_data& gd, Graph& graph) {
    for (int i = 0; i < gd.size(); ++i) {
        std::vector<Node> new_column;
        graph.push_back(new_column);
        for (int j = 0; j < gd[i].size(); ++j) {
            Node new_node = {i, j, &gd[i][j]};
           graph[i].push_back(new_node);
        }
    }
}

float cost_function(Node n1, Node n2) {
    return fabs(*n1.value - *n2.value);
}

std::vector<Node> get_neighbors(Node node, Graph& graph) {
    std::vector<Node> result;
    // check if we are at the end of the graph
    if (node.path_index == graph.size()) {
        std::vector<Node> empty;
        return result; // empty vector
    } else {
        for (auto n : graph[node.path_index + 1]) {
            result.push_back(n);
        }
    }
}

void print_nodes(std::vector<Node> nodes) {
    using namespace std;
    for (auto node : nodes) {
        cout << "(" << node.path_index << ", ";
        cout << node.sample_index << ")\n";
    }
    cout << endl;
}

void read_file(std::string filename, graph_data& gd) {
    // open file
    std::ifstream infile(filename);

    // check if opened succesfull
    if (!infile) std::cout << "Failed to open file." << std::endl;

    // read line by line and save in temporary string "line"
    std::string line;
    int i = 0;
    while(infile) {
        getline(infile, line);
        // the last line is an empty string apparantly
        if (line != "") {
            // use stringstream object to parse a single line
            std::stringstream ss(line);
            std::string temp;
            // add new column so I can do gd[i] assignment
            std::vector<float> new_column;
            gd.push_back(new_column);
            while (getline(ss, temp, ',')) {
                // convert to float and print
                gd[i].push_back(stof(temp));
            }
            ++i;
        }
    }
    // file is closed when infile goes out of scope
}

void print_test_data(graph_data& gd) {
    using namespace std;
    cout << "The test data: \n";
    for (auto pt : gd ) {
        cout << "-------------\n";
        for (auto val : pt) {
            cout << val << endl;
        }
    }
    cout << "-------------" << endl;
}