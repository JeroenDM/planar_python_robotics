#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

const float BIG = 9999;

struct Node {
    int path_index;
    int sample_index;
    float* value;
    float dist;
    Node* parent;
};
Node DUMMY_NODE;

typedef std::vector<std::vector<float>> graph_data;
typedef std::vector<std::vector<Node>> graph;

void read_file(std::string filename, graph_data& gd);
void add_data_to_graph(graph_data& gd, graph& g);

namespace print {
    void test_data(graph_data& gd);
    void nodes(std::vector<Node>);
    void nodes(std::vector<Node*>);
    void node(Node*);
    void test_graph(graph&);
}

//===========================================
// data handling functions
//===========================================
void add_data_to_graph(graph_data& gd, graph& graph) {
    for (int i = 0; i < gd.size(); ++i) {
        std::vector<Node> new_column;
        graph.push_back(new_column);
        for (int j = 0; j < gd[i].size(); ++j) {
            Node new_node = {i, j, &gd[i][j], BIG, &DUMMY_NODE};
           graph[i].push_back(new_node);
        }
    }
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
    }
    // file is closed when infile goes out of scope
}

//===========================================
// print functions for debugging
//===========================================

void print::nodes(std::vector<Node> nodes) {
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

void print::nodes(std::vector<Node*> nodes) {
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

void print::node(Node* node) {
    using namespace std;
    cout << "(" << (*node).path_index << ", ";
        cout << (*node).sample_index << ")";
        cout << " dist: " << (*node).dist;
        cout << "\tparent: ";
        cout << "(" << (*(*node).parent).path_index << ", ";
        cout << (*(*node).parent).sample_index << ")\n";
        cout << endl;
}

void print::test_data(graph_data& gd) {
    std::cout << "The test data: \n";
    for (auto pt : gd ) {
        std::cout << "-------------\n";
        for (auto val : pt) {
            std::cout << val << std::endl;
        }
    }
    std::cout << "-------------" << std::endl;
}

void print::test_graph(graph &g) {
    for (auto col : g) {
        std::cout << "--------\n";
        print::nodes(col);
    }
    std::cout << "--------" << std::endl;
}