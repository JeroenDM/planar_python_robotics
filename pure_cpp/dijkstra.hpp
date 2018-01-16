#include <iostream>
#include <vector>
#include <cmath> // fabs
#include <algorithm>    // std::sort

namespace graph {
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
    // the graph is the only variable containing the nodes
    // all the other containers with nodes
    // contain pointers to the nodes in graph
    typedef std::vector<std::vector<Node>> Graph;

    void add_data_to_graph(graph_data& gd, Graph& g);
    void init_unvisited(std::vector<Node*>&, Graph& g);
    float cost_function(Node, Node);
    std::vector<Node*> get_neighbors(Node&, Graph& g);
    void visit(Node*, Graph&);
    bool sort_function(Node*, Node*);
    void run_dijkstra(Node* start, Graph& g);
    std::vector<Node*> get_path(Node*);

    // util functions
    void print_test_data(graph_data& gd);
    void print_nodes(std::vector<Node>);
    void print_nodes(std::vector<Node*>);
    void print_nodes(Node*);


    //===========================================
    // implementation graph functions
    //===========================================

    void run_dijkstra(Node* start_node, Graph& g) {
        using namespace std;
        // sorting the whole unvisited set is inefficient
        std::vector<Node*> visited;
        std::vector<Node*> unvisited;

        // add pointers to all notes to unvisited
        // nodes of first column that are not added
        init_unvisited(unvisited, g);

        // start node stuff
        (*start_node).dist = 0;
        unvisited.push_back(start_node);

        cout << "Before dijkstra" << endl;
        print_nodes(unvisited);
        print_nodes(visited);

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
                visit(current, g);
                std::sort(unvisited.begin(), unvisited.end(), sort_function);
                visited.push_back(current);
            } else {
                cout << "Alle nodes visited!" << endl;
                break;
            }      
        }
        if (i == MAX_ITER) cout << "Maximum iterations reached" << endl;

        cout << "after loop" << endl;
        print_nodes(unvisited);
        print_nodes(visited);
    }

    void init_unvisited(std::vector<Node*>& uv, Graph& g) {
        // do not add fist column (start at i=1)
        // start node in first column cannot reached the other nodes there
        for (int i = 1; i < g.size(); ++i) {
            for (Node& n : g[i]) {
                uv.push_back(&n);
            }
        }
    }

    std::vector<Node*> get_path(Node* goal) {
        std::vector<Node*> path;
        Node* current_node = goal;
        while ((*current_node).path_index > 0) {
            path.push_back(current_node);
            current_node = (*current_node).parent;
        }
        path.push_back(current_node);
        return path;
    }

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
    // implementation util functions
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
} // namespace graphs