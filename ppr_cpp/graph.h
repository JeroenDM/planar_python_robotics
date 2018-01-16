#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <vector>
//#include <Eigen/Dense>

void input_matrix(double* mat, int nrows, int ncols);

// fixed length container more efficient but less flexible
typedef std::vector<double> joint_value;

// graph functions
struct Node {
    int path_index;
    int sample_index;
    //double* value;
    joint_value* jv;
    double dist;
    Node* parent;
};

typedef std::vector< std::vector<joint_value> > graph_data;
// the graph is the only variable containing the nodes
// all the other containers with nodes
// contain pointers to the nodes in graph
typedef std::vector< std::vector<Node> > node_array;

const float INF = 9999;

bool sort_function(Node* n1, Node* n2);

class Graph {
    graph_data gd;
    node_array na;
    void graph_data_to_node_array();
    void init_unvisited(std::vector<Node*>& uv);
    float cost_function(Node n1, Node n2);
    void visit(Node* node);
    void dijkstra_algorithm(Node* start_node);
    void print_nodes(std::vector<Node*> nodes);
    void print_nodes(std::vector<Node> nodes);
    std::vector<Node*> get_neighbors(Node* node);
    std::vector<Node*> get_path_nodes();
public:
    void add_data_column(double* mat, int nrows, int ncols);
    void print_graph_data();
    void init_dijkstra();
    void run_dijkstra();
    void get_path(int* vec, int n);
    void print_path();
    void print_graph();
};


#endif