#ifndef _GRAPH_H_
#define _GRAPH_H_

#include <vector>
//#include <Eigen/Dense>

void input_matrix(double* mat, int nrows, int ncols);

// graph functions
struct Node {
    int path_index;
    int sample_index;
    double* value;
    double dist;
    Node* parent;
};

typedef std::vector< std::vector<double> > graph_data;
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
public:
    void add_data_column(double* vec, int n);
    void print_graph_data();
    void init_dijkstra();
    void run_dijkstra();
    void print_path();
    void print_graph();
};


#endif