#include <iostream>
#include <fstream>
#include <sstream> 
//#include <vector>

#include "dijkstra.h" // inludes graph.h and std::vector

//===========================================
// MAIN
//===========================================
int main() {
    using namespace std;
    cout << "Testing graph class\n";
    cout << "-------------------" << endl;

    graph_data data;
    read_file("simple_example.txt", data);
    print::test_data(data);

    graph g;
    add_data_to_graph(data, g);
    print::test_graph(g);

    Dijkstra dk;
    dk.init(g, &g[0][0]);

    //vector<Node*> nb = dk.get_neighbors(dk.start);
    //print::nodes(nb);

    return 0;
}

