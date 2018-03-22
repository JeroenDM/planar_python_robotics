#include <iostream>
#include <vector>
#include "graph.h"

using namespace std;

void test_graph_creation();
void test_sort_function();
void test_bfs();

int main() {
    cout << "============================\n";
    cout << "Running tests for graph.cxx\n";
    cout << "============================\n";
    cout << endl;

    cout << "------------------------------\n";
    cout << "test_sort_function\n";
    test_sort_function();
    cout << endl;
    cout << "------------------------------\n";
    cout << "test_graph_creation\n";
    test_graph_creation();

    cout << "------------------------------\n";
    cout << " test bfs\n";
    test_bfs();
    cout << endl;

    return 0;
}

void test_graph_creation() {
    graph_data data = {{{0, 0}, {0, 1}},
                        {{1, -1}, {1, 0}, {1, 1}},
                        {{0, 2}, {2, 2}} };

    Graph g;
    g.set_graph_data(data);
    //g.print_graph_data();
    g.init_dijkstra();
    g.run_dijkstra();
    g.print_path();
}

void test_bfs() {
    graph_data data = {{{0, 0}, {0, 1}},
                        {{1, -1}, {1, 0}, {1, 1}},
                        {{0, 2}, {2, 2}} };

    Graph g;
    g.set_graph_data(data);
    //g.print_graph_data();
    g.init_dijkstra();
    g.run_bfs();
    g.print_path();
}

void test_sort_function() {
    Node n1, n2;
    n1.dist = 3.0;
    n2.dist = 2.0;

    bool res = sort_function(&n1, &n2);

    cout << "Result: " << res;
}