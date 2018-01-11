#include <iostream>
#include <lemon/list_graph.h>
#include <lemon/dijkstra.h>
#include <vector>

using namespace lemon;
using namespace std;

int main() {

    ListDigraph g;

    // add nodes
    vector<ListDigraph::Node> nodes;
    ListDigraph::NodeMap<int> label(g);
    for (int i = 0; i<5; ++i) {
        nodes.push_back(g.addNode());
        label[nodes[i]] = i;
    }

    // add arcs
    ListDigraph::Arc a1, a2, a3, a4;
    a1 = g.addArc(nodes[0], nodes[1]);
    a2 = g.addArc(nodes[1], nodes[3]);
    a3 = g.addArc(nodes[0], nodes[2]);
    a4 = g.addArc(nodes[2], nodes[3]);

    // add cost map
    ListDigraph::ArcMap<int> cost(g);
    cost[a1] = 4;
    cost[a2] = 4;
    cost[a3] = 5;
    cost[a4] = 2;

    Path<ListDigraph> p;
    ListDigraph::NodeMap<int> dist(g);
    // dijkstra(g, cost).distMap(dist).path(p).run(nodes[0], nodes[3]);
    Dijkstra<ListDigraph> dk(g, cost);
    dk.distMap(dist);
    dk.init();
    dk.run(nodes[0], nodes[3]);

    // minimum distance to the last node
    cout << "Shortest path length: \n";
    cout << dist[nodes[3]] << endl;

    // go from last node to start and put the nodes in a vector
    vector<ListDigraph::Node> path;
    for (ListDigraph::Node n = nodes[3]; n != nodes[0]; n = dk.predNode(n)) {
        path.push_back(n);
    }
    path.push_back(nodes[0]);

    // reverse path vector
    reverse(path.begin(), path.end());

    // the final path
    cout << "The path: \n";
    for (auto p : path) {
        cout << label[p] << " ";
    }
    cout << endl;

    return 0;
}