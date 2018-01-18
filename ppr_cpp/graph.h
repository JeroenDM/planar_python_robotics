#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <vector>

// fixed length container more efficient but less flexible
// the length is the number of dofs of the robot, which can vary
typedef std::vector<float> joint_value;

// graph functions
struct Node {
    int path_index;
    int sample_index;
    joint_value* jv;
    float dist;
    Node* parent;
};

// the graph_data is the only place where the joint values are stored
// (exept in python)
// Nodes containt pointers to the data
typedef std::vector< std::vector<joint_value> > graph_data;
// the node_array is the only variable containing the nodes
// all the other containers with nodes
// contain pointers to the nodes in graph
typedef std::vector< std::vector<Node> > node_array;

// sort function for dijkstra unvisited list
bool sort_function(Node* n1, Node* n2);

class Graph {
    graph_data gd;
    node_array na;
    int MAX_ITER = 10000;

    void               graph_data_to_node_array();
    void               init_unvisited(std::vector<Node*>& uv);
    float              cost_function(Node n1, Node n2);
    void               visit(Node* node);
    std::vector<Node*> get_neighbors(Node* node);
    std::vector<Node*> get_path_nodes();
    void               dijkstra_algorithm(Node* start_node);

    void print_node(Node n);
    void print_nodes(std::vector<Node*> nodes);
    void print_nodes(std::vector<Node> nodes);

public:
    void add_data_column(float* mat, int nrows, int ncols);
    void print_graph_data();
    void init_dijkstra();
    void run_dijkstra();
    void get_path(int* vec, int n);
    void print_path();
    void print_graph();
};


#endif