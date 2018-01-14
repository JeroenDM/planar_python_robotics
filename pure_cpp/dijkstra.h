//#include <vector>
#include "graph.h" // inludes std::vector

//===========================================
// graph class implementation
//===========================================
class Dijkstra {
public:
    graph g;
    Node* start;
    int graph_width; // number of path points
    std::vector<Node*> visited;
    std::vector<Node*> unvisited;

    void init(graph g, Node* start);
    bool run();
    std::vector<Node*> get_path();
    std::vector<Node*> get_neighbors(Node*);
};

//===========================================
// graph class implementation (should not be here)
//===========================================
void Dijkstra::init(graph g, Node* start) {
    g = g;
    start = start;
}
std::vector<Node*> Dijkstra::get_neighbors(Node* node) {
    std::vector<Node*> result;
    // check if we are at the end of the graph
    if ((*node).path_index == g.size()) {
        std::vector<Node*> empty;
        return result; // empty vector
    } else {
        for (Node n : g[(*node).path_index + 1]) {
            result.push_back(&n);
        }
    }
}