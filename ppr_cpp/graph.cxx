#include <iostream>
#include <algorithm>
#include <limits> // infinity()
#include "graph.h"

// dummy node to initialize parent nodes
Node DUMMY_NODE;

// to initialize distances for dijkstra
const float INF = std::numeric_limits<float>::infinity();

// sort function for dijkstra unvisited list
bool sort_function(Node* n1, Node* n2) {
    return (*n2).dist < (*n1).dist;
}

// ===========================================================
// MISTER DIJKSTRA
// ===========================================================
// returns status int
//  1 : success
// -1 : maximum itterations reached
int Graph::dijkstra_core(std::vector<Node*>& U, std::vector<Node*>& V) {
    Node* current;
    // keep track of unvisited nodes with updated distance
    // this allows you to only sort the relevant part of unvisited
    int i;
    for (i = 0; i<MAX_ITER; ++i) {
        if (U.size() > 0) {
            current = U.back();
            if ((*current).dist == INF ) {
                //only unreachable nodes left
                break;
            }
            U.pop_back();
            visit(current);
            std::sort(U.begin(), U.end(), sort_function);
            V.push_back(current);
        } else {
            return 2;
        }      
    }
    if (i == MAX_ITER) {
        return -1;
    } else {
        return 1;
    }
}

int Graph::bfs_core(std::vector<Node*>& U) {
    for (Node* n : U) {
        visit(n);
    }
    return 1;
}

void Graph::bfs(Node* start_node) {
    using namespace std;
    // sorting the whole unvisited set is inefficient
    vector<Node*> unvisited;

    // add pointers to all notes to unvisited
    // nodes of first column that are not added
    init_unvisited(unvisited);

    // start node stuff
    (*start_node).dist = 0;
    unvisited.push_back(start_node);

    // DEBUG stuff
    // cout << "Before dijkstra" << endl;
    // print_nodes(unvisited);
    // print_nodes(visited);

    // run the actual algorithm
    int stat = bfs_core(unvisited);

    if (stat == 1) {
        cout << "BFS finished" << endl;
        path_found = true;
    } else {
        cout << "Unkown status" << endl;
    }

    //DEBUG stuff
    cout << "After BFS" << endl;
    print_graph();
}


void Graph::single_source_dijkstra(Node* start_node){
    using namespace std;
    // sorting the whole unvisited set is inefficient
    vector<Node*> visited;
    vector<Node*> unvisited;

    // add pointers to all notes to unvisited
    // nodes of first column that are not added
    init_unvisited(unvisited);

    // start node stuff
    (*start_node).dist = 0;
    unvisited.push_back(start_node);

    // DEBUG stuff
    // cout << "Before dijkstra" << endl;
    // print_nodes(unvisited);
    // print_nodes(visited);

    // run the actual algorithm
    int stat = dijkstra_core(unvisited, visited);

    if (stat == -1) {
        cout << "Maximum iterations reached" << endl;
    } else if (stat == 1) {
        cout << "Dijkstra finished succesfully" << endl;
        path_found = true;
    } else if (stat == 2) {
        cout << "Dijkstra finished succesfully" << endl;
        cout << "All nodes where visited" << endl;
        path_found = true;
    } else {
        cout << "Unkown status" << endl;
    }

    // DEBUG stuff
    // cout << "After dijkstra" << endl;
    // print_nodes(unvisited);
    // print_nodes(visited);
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
    float cost = 0;
    int s = (*n1.jv).size();
    for (int i = 0; i<(*n1.jv).size(); ++i) {
        cost += fabs( (*n1.jv)[i] - (*n2.jv)[i] );
    }
    return cost;
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

std::vector<Node*> Graph::get_path_nodes() {
    std::vector<Node*> path;

    if (path_found) {
        // find last node with shortest distance to start
        float min_dist = INF;
        Node* goal;
        for (auto& n : na.back()) {
            if (n.dist < min_dist) {
                goal = &n;
                min_dist = n.dist;
            }
        }

        Node* current_node = goal;
        while ((*current_node).path_index > 0) {
            path.push_back(current_node);
            current_node = (*current_node).parent;
        }
        path.push_back(current_node);
        std::reverse(path.begin(), path.end());
        return path;
    }
    else {
        std::cout << "No path found" << std::endl;
        return path;
    }
}

// ===========================================================
// puplic graph fucntions
// ===========================================================
// rows containing the different solutions
// columns different joint values
void Graph::add_data_column(float* mat, int nrows, int ncols) {
    int index;
    std::vector<joint_value> new_col;
    for (int i=0; i<nrows; ++i) {
        joint_value new_jv;
        for (int j=0; j<ncols; ++j) {
            index = index = j + ncols*i;
            new_jv.push_back(mat[index]);
        }
        new_col.push_back(new_jv);
    }
    gd.push_back(new_col);
}

void Graph::init_dijkstra() {
    std::cout << "Initializing graph for planning" << std::endl;
    graph_data_to_node_array();
    path_found = false;
}

void Graph::run_dijkstra() {
    std::cout << "Running dijkstra's algorithm" << std::endl;
    single_source_dijkstra(&na[0][0]);
    //bfs(&na[0][0]);
}

void Graph::get_path(int* vec, int n) {
    if (path_found) {
        std::vector<Node*> path;
        path = get_path_nodes();
        if (n != path.size()) {
            std::cout << "Wrong path length" << std::endl;
        }
        else {
            for (int i = 0; i<n; ++i) {
                vec[i] = (*path[i]).sample_index;
            }
        }
    }
    else {
        std::cout << "No path found" << std::endl;
        for (int i = 0; i<n; ++i) {
                vec[i] = -1;
        }
    }
}

// ===========================================================
// print functions for debugging
// ===========================================================
// private
void Graph::print_node(Node n) {
    using namespace std;
    cout << "(" << n.path_index << ", ";
    cout << n.sample_index << ")";
    cout << " dist: " << n.dist;
    cout << " parent: ";
    cout << "(" << (*n.parent).path_index << ", ";
    cout << (*n.parent).sample_index << ")\n";
}

void Graph::print_nodes(std::vector<Node*> nodes) {
    using namespace std;
    for (Node* node : nodes) {
        print_node(*node);
    }
    cout << endl;
}

void Graph::print_nodes(std::vector<Node> nodes) {
    using namespace std;
    for (Node& node : nodes) {
        print_node(node);
    }
    cout << endl;
}

//public
void Graph::print_graph_data() {
    using namespace std;
    for(auto& col : gd) {
        cout << "----------------" << endl;
        for (auto& val : col) {
            for (float& d : val) {
                cout << d << " ";
            }
            cout << endl;
        }
    }
}

void Graph::print_path() {
    using namespace std;
    cout << "The most recent shortest path is:" << endl;
    print_nodes(get_path_nodes());
}

void Graph::print_graph() {
    for (auto& p : na) {
        print_nodes(p);
    }
}