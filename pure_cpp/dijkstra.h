//#include <vector>
#include "graph.h" // inludes std::vector
#include <cmath> // fabs
#include <algorithm> // sort

// TODO remove this global function ?
// https://stackoverflow.com/questions/29286863/invalid-use-of-non-static-member-function
bool sort_function(Node* n1, Node* n2) {
    return (*n2).dist < (*n1).dist;
}

//===========================================
// graph class implementation
//===========================================
class Dijkstra {
    graph g;
    Node* start;
    // sorting the whole unvisited set is inefficient
    std::vector<Node*> visited;
    std::vector<Node*> unvisited;
    const int MAX_ITER = 10;
public:
    Dijkstra(graph g, int i);

    // algorithm functions
    void init();
    std::vector<Node*> get_neighbors(Node* node);
    float cost_function(Node*, Node*);
    void visit(Node*);
    //bool sort_function(Node*, Node*);
    void run();

    // debug functions
    void show_graph();
    void test();
};

//===========================================
// graph class implementation (should not be here)
//===========================================
Dijkstra::Dijkstra(graph in_g, int i) {
    g = in_g;
    start = &g[0][i];
    (*start).dist = 0;
    init();
}

void Dijkstra::init() {
    // add pointers to all notes to the unvisited set
    // nodes of first column that are not added
    // they cannot be reached by a node from the same column
    // (assuming the start node is of the first column)
    std::cout << "Unvisited set initialized" << std::endl;
    for (auto& p : g) {
        for (Node& n : p) {
            if (n.path_index > 0) unvisited.push_back(&n); // ugly way
        }
    }
    unvisited.push_back(start);
}

std::vector<Node*> Dijkstra::get_neighbors(Node* node) {
    std::vector<Node*> result;
    // check if we are at the end of the graph
    if ((*node).path_index == g.size()) {
        std::vector<Node*> empty;
        return result; // empty vector
    } else {
        for (Node& n : g[(*node).path_index + 1]) {
            result.push_back(&n);
        }
    }
}

float Dijkstra::cost_function(Node* n1, Node* n2) {
    return std::fabs(*(*n1).value - *(*n2).value);
}

void Dijkstra::visit(Node* n) {
    std::vector<Node*> neighbors;
    neighbors = get_neighbors(n);
    for (Node* nb : neighbors) {
        float dist = (*n).dist + cost_function(nb, n);
        if (dist < (*nb).dist) {
            (*nb).dist = dist;
            (*nb).parent = n;
        }
    }
}

// bool Dijkstra::sort_function(Node* n1, Node* n2) {
//     return (*n2).dist < (*n1).dist;
// }

void Dijkstra::run() {
    Node* current;
    // sort from big dist to small
    // start node has dist = 0 and will be at the back
    std::sort(unvisited.begin(), unvisited.end(), sort_function);
    for (int i = 0; i<MAX_ITER; ++i) {
        if (unvisited.size() > 0) {
            current = unvisited.back();
            unvisited.pop_back();
            visit(current);
            std::sort(unvisited.begin(), unvisited.end(), sort_function);
            visited.push_back(current);
        } else {
            std::cout << "Alle nodes visited!" << std::endl;
            break;
        }
    }
}


//===========================================
// graph class debug functions
//===========================================
void Dijkstra::show_graph() {
    using namespace std;
    cout << "Dijkstra: showing graph" << endl;
    cout << g.size() << endl;
    for (auto col : g) {
        cout << "col: " << col[0].path_index << endl;
        print::nodes(col);
    }
}

void Dijkstra::test() {
    using namespace std;
    //cout << "Testing get_neighbors" << endl;
    //std::vector<Node*> nb = get_neighbors(start);
    //cout << "There are " << nb.size() << " neighbors." << endl;
    //print::nodes(nb);
    print::nodes(unvisited);
    print::nodes(visited);

    // float c = cost_function(start, start);
    // cout << "Cost: " << c << endl;
    // visit(start);
    run();

    cout << "after start node is visited" << endl;
    print::nodes(unvisited);
    print::nodes(visited);
}