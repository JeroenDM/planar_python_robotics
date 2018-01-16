#include <iostream>
#include <algorithm>
#include "graph.h"

using namespace std;
void input_matrix(double* mat, int nrows, int ncols) {
    cout << "Executing input_matrix" << endl;

    graph_data gd;
    int index = 0;
    for (int i=0; i<nrows; ++i) {
        for (int j=0; j<ncols; ++j) {
            index = j + ncols*i;
            cout << mat[index];
        }
    }
    cout << "\n-------------------" << endl;
}

Node DUMMY_NODE;

bool sort_function(Node* n1, Node* n2) {
    return (*n2).dist < (*n1).dist;
}

// ===========================================================
// private graph fucntions
// ===========================================================
void Graph::graph_data_to_node_array() {
    for (int i = 0; i < gd.size(); ++i) {
            std::vector<Node> new_column;
            na.push_back(new_column);
            for (int j = 0; j < gd[i].size(); ++j) {
                Node new_node = {i, j, &gd[i][j], INF, &DUMMY_NODE};
                na[i].push_back(new_node);
            }
        }
}

void Graph::dijkstra_algorithm(Node* start_node){
    using namespace std;
    // sorting the whole unvisited set is inefficient
    std::vector<Node*> visited;
    std::vector<Node*> unvisited;

    // add pointers to all notes to unvisited
    // nodes of first column that are not added
    init_unvisited(unvisited);

    // start node stuff
    (*start_node).dist = 0;
    unvisited.push_back(start_node);

    cout << "Before dijkstra" << endl;
    print_nodes(unvisited);
    print_nodes(visited);

    // Node* current;
    // int i;
    // for (i = 0; i<4; ++i) {
    //     current = unvisited.back();
    //     unvisited.pop_back();

    //     visit(current);
    //     std::sort(unvisited.begin(), unvisited.end(), sort_function);

    //     visited.push_back(current);

    //     cout << "During dijkstra---------------" << endl;
    //     print_nodes(unvisited);
    //     print_nodes(visited);
    // }

    Node* current;
    // keep track of unvisited nodes with updated distance
    // this allows you to only sort the relevant part of unvisited
    // int nodes_seen_cntr = 0;
    const int MAX_ITER = 10;
    int i;
    for (i = 0; i<MAX_ITER; ++i) {
        if (unvisited.size() > 0) {
            current = unvisited.back();
            unvisited.pop_back();
            visit(current);
            std::sort(unvisited.begin(), unvisited.end(), sort_function);
            visited.push_back(current);
        } else {
            cout << "Alle nodes visited!" << endl;
            break;
        }      
    }
    if (i == MAX_ITER) cout << "Maximum iterations reached" << endl;

    cout << "After dijkstra" << endl;
    print_nodes(unvisited);
    print_nodes(visited);
}

void Graph::init_unvisited(std::vector<Node*>& uv) {
    // do not add fist column (start at i=1)
    // start node in first column cannot reached the other nodes there
    for (int i = 1; i < na.size(); ++i) {
        for (Node& n : na[i]) {
            uv.push_back(&n);
        }
    }
}

float Graph::cost_function(Node n1, Node n2) {
    return fabs(*n1.value - *n2.value);
}

std::vector<Node*> Graph::get_neighbors(Node* node) {
    std::vector<Node*> result;
    // check if we are at the end of the graph
    if ((*node).path_index == (na.size()-1)) {
        std::vector<Node*> empty;
        return result; // empty vector
    } else {
        for (Node& n : na[(*node).path_index + 1]) {
            result.push_back(&n);
        }
    }
}

void Graph::visit(Node* n) {
    std::vector<Node*> neighbors;
    neighbors = get_neighbors(n);
    for (Node* nb : neighbors) {
        float dist = (*n).dist + cost_function(*nb, *n);
        if (dist < (*nb).dist) {
            (*nb).dist = dist;
            (*nb).parent = n;
        }
    }
}

void Graph::print_nodes(std::vector<Node*> nodes) {
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

void Graph::print_nodes(std::vector<Node> nodes) {
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

// ===========================================================
// puplic graph fucntions
// ===========================================================
void Graph::add_data_column(double* vec, int n) {
    std::vector<double> new_column;
    gd.push_back(new_column);
    for (int i=0; i<n; ++i) {
        gd.back().push_back(vec[i]);
    }
}
void Graph::print_graph_data() {
    for(auto& col : gd) {
        for (double& d : col) {
            cout << d << endl;
        }
    }
}

void Graph::init_dijkstra() {
    cout << "Initializing graph for planning" << endl;
    graph_data_to_node_array();
}

void Graph::run_dijkstra() {
    cout << "Running dijkstra's algorithm" << endl;
    dijkstra_algorithm(&na[0][0]);
}

void Graph::print_path() {
    cout << "The most recent shortest path is:" << endl;
    // find last node with shortest distance to start
    double min_dist = INF;
    Node* goal;
    for (auto& n : na.back()) {
        if (n.dist < min_dist) {
            goal = &n;
            min_dist = n.dist;
        }
    }

    std::vector<Node*> path;
    Node* current_node = goal;
    while ((*current_node).path_index > 0) {
        path.push_back(current_node);
        current_node = (*current_node).parent;
    }
    path.push_back(current_node);
    print_nodes(path);
}

void Graph::print_graph() {
    for (auto& p : na) {
        print_nodes(p);
    }
}