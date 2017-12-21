#include <iostream>
#include <string>
#include <map>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/array.hpp>
#include <array>
#include <utility>

using namespace std;

int main() {
    map<string, int> book;
    book.insert(make_pair("John", 10));
    book.insert(make_pair("Elvis", 4));

    cout << book["John"] << endl;

    for (auto i : book) {
        cout << i.first << " : " << i.second << "\n";
    }

    cout << "--------------------\n";

    // shorthand name for the types in the map
    typedef string name;
    typedef int number;

    name jane = "Jane";
    number n_jane = 55;
    book.insert(make_pair(jane, n_jane));

    // boost property map
    boost::associative_property_map<map<string, int>> book_map(book);
    typedef boost::associative_property_map<map<string, int>> book_map_type;

    typedef typename boost::property_traits<book_map_type>::value_type value_type;
    typedef typename boost::property_traits<book_map_type>::key_type key_type;

    key_type aug = "August";
    value_type n_aug = 8;
    book.insert(make_pair(aug, n_aug));

    for (auto i : book) {
        cout << i.first << " : " << i.second << "\n";
    }

    // shortest path example
    enum { topLeft, topRight, bottomRight, bottomLeft };

    std::array<std::pair<int, int>, 4> edges{{
        std::make_pair(topLeft, topRight),
        std::make_pair(topRight, bottomRight),
        std::make_pair(bottomRight, bottomLeft),
        std::make_pair(bottomLeft, topLeft)
    }};

    typedef boost::adjacency_list<boost::listS, boost::vecS,
    boost::undirectedS, boost::no_property,
    boost::property<boost::edge_weight_t, int>> graph;

    std::array<int, 4> weights{{2, 1, 1, 1}};

    graph g{edges.begin(), edges.end(), weights.begin(), 4};

    boost::array<int, 4> directions;
    boost::dijkstra_shortest_paths(g, bottomRight,
    boost::predecessor_map(directions.begin()));

    int p = topLeft;
    while (p != bottomRight)
    {
        std::cout << p << '\n';
        p = directions[p];
    }
    std::cout << p << '\n';

}