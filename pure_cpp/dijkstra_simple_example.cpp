#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>      // std::stringstream (also in Eigen?)
// #include <Eigen/Dense>
#include <cmath> // fabs ??
#include <algorithm>    // std::sort

const float INF = 9999;

// graph functions
struct Node {
    int path_index;
    int sample_index;
    float* value;
    float dist;
    Node* parent;
};
Node DUMMY_NODE;
typedef std::vector<std::vector<float>> graph_data;
typedef std::vector<std::vector<Node>> Graph;

void add_data_to_graph(graph_data& gd, Graph& g);
float cost_function(Node, Node);
std::vector<Node*> get_neighbors(Node&, Graph& g);
void visit(Node*, Graph&);
bool sort_function(Node*, Node*);

// util functions
void read_file(std::string filename, graph_data& gd);
void print_test_data(graph_data& gd);
void print_nodes(std::vector<Node>);
void print_nodes(std::vector<Node*>);
void print_nodes(Node*);

//===========================================
// MAIN
//===========================================

int main() {
    using namespace std;
    //using namespace Eigen;

    // read random test data from text file
    vector<float> test;
    
    graph_data data;
    read_file("simple_example.txt", data);
    //print_test_data(data);

    // test Node structure
    Node node1 = {0, 0, &data[0][0]};
    Node node2 = {1, 0, &data[1][0]};
    vector<Node> ns = {node1, node2};
    // print_nodes(ns);
    // cout << cost_function(node1, node2) << endl;

    Graph g;
    add_data_to_graph(data, g);
    // for (auto col : g) {
    //     cout << "--------\n";
    //     print_nodes(col);
    // }
    // cout << "--------" << endl;

    //vector<Node> nb;
    //nb = get_neighbors(g[0][0], g);
    // print_nodes(nb);

    // sorting the whole unvisited set is inefficient
    vector<Node*> visited;
    vector<Node*> unvisited;

    // add pointers to all notes to unvisited
    // nodes of first column that are not added
    // they cannot be reached by a node from the same column
    for (auto &p : g) {
        for (Node& n : p) {
            if (n.path_index > 0) unvisited.push_back(&n);
        }
    }

    //print_nodes(unvisited);

    // vist first Node
    Node* start_node = &g[0][0];
    (*start_node).dist = 0;
    visit(start_node, g);

    //print_nodes(unvisited);

    // the end iterator can maybe be smaller
    // depending on which distance is already up to date
    std::sort(unvisited.begin(), unvisited.end(), sort_function);
    // when sorted, the last element is the one just visited
    visited.push_back(start_node);

    cout << "after start node is visited" << endl;
    print_nodes(unvisited);
    print_nodes(visited);

    Node* current;
    //int iter = 0;
    const int MAX_ITER = 10;
    for (int i = 0; i<MAX_ITER; ++i) {
        if (unvisited.size() > 0) {
            current = unvisited.back();
            unvisited.pop_back();
            visit(current, g);
            std::sort(unvisited.begin(), unvisited.end(), sort_function);
            visited.push_back(current);
        } else {
            cout << "Alle nodes visited!" << endl;
            break;
        }       
    }

    cout << "after loop" << endl;
    print_nodes(unvisited);
    print_nodes(visited);

    return 0;
}

//===========================================
// declaration graph functions
//===========================================

bool sort_function(Node* n1, Node* n2) {
    return (*n2).dist < (*n1).dist;
}

void visit(Node* n, Graph& g) {
    std::vector<Node*> neighbors;
    neighbors = get_neighbors(*n, g);
    for (Node* nb : neighbors) {
        float dist = (*n).dist + cost_function(*nb, *n);
        if (dist < (*nb).dist) {
            (*nb).dist = dist;
            (*nb).parent = n;
        }
    }
}

void add_data_to_graph(graph_data& gd, Graph& graph) {
    for (int i = 0; i < gd.size(); ++i) {
        std::vector<Node> new_column;
        graph.push_back(new_column);
        for (int j = 0; j < gd[i].size(); ++j) {
            Node new_node = {i, j, &gd[i][j], INF, &DUMMY_NODE};
           graph[i].push_back(new_node);
        }
    }
}

float cost_function(Node n1, Node n2) {
    return fabs(*n1.value - *n2.value);
}

std::vector<Node*> get_neighbors(Node& node, Graph& graph) {
    std::vector<Node*> result;
    // check if we are at the end of the graph
    if (node.path_index == graph.size()) {
        std::vector<Node*> empty;
        return result; // empty vector
    } else {
        for (Node& n : graph[node.path_index + 1]) {
            result.push_back(&n);
        }
    }
}

//===========================================
// declaration util functions
//===========================================

void print_nodes(std::vector<Node> nodes) {
    using namespace std;
    for (auto node : nodes) {
        cout << "(" << node.path_index << ", ";
        cout << node.sample_index << ")";
        cout << " dist: " << node.dist;
        cout << " parent: ";
        cout << "(" << (*node.parent).path_index << ", ";
        cout << (*node.parent).sample_index << ")\n";
    }
    cout << endl;
}

void print_nodes(std::vector<Node*> nodes) {
    using namespace std;
    for (auto node : nodes) {
        cout << "(" << (*node).path_index << ", ";
        cout << (*node).sample_index << ")";
        cout << " dist: " << (*node).dist;
        cout << "\tparent: ";
        cout << "(" << (*(*node).parent).path_index << ", ";
        cout << (*(*node).parent).sample_index << ")\n";
    }
    cout << endl;
}

void print_nodes(Node* node) {
    using namespace std;
    cout << "(" << (*node).path_index << ", ";
        cout << (*node).sample_index << ")";
        cout << " dist: " << (*node).dist;
        cout << "\tparent: ";
        cout << "(" << (*(*node).parent).path_index << ", ";
        cout << (*(*node).parent).sample_index << ")\n";
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
        // getline(infile, line);
        // std::cout << "reading line: " << line << std::endl;
        // // the last line is an empty string apparantly
        // if (line != "") {
        //     // use stringstream object to parse a single line
        //     std::stringstream ss(line);
        //     std::string temp;
        //     // add new column so I can do gd[i] assignment
        //     std::vector<float> new_column;
        //     gd.push_back(new_column);
        //     while (getline(ss, temp, ',')) {
        //         // convert to float and print
        //         gd[i].push_back(stof(temp));
        //     }
        //     ++i;
        // }
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