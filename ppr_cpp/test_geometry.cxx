#include <iostream>
#include <vector>
#include "geometry.h"

void printv(std::vector<point> v) {
    for (int i=0; i < v.size(); ++i) {
        std::cout << "element " << i << " \n";
        std::cout << v[i] << std::endl;
    }
}

int main() {
    std::cout << "test the rectangle class" << std::endl;
    Rectangle rec(1, 2, 0, 0, 0.2);
    std::cout << rec.area() << std::endl;
    std::cout << rotation(1.6) << std::endl;
    printv(rec.get_coordinates());
    printv(rec.get_normals());
}