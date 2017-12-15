#include <iostream>
#include <vector>
#include "geometry.h"

void printv(std::vector<point> v) {
    for (int i=0; i < v.size(); ++i) {
        std::cout << "---------- " << i << " \n";
        std::cout << v[i] << std::endl;
    }
}

void printv(std::vector<float> v) {
    for (int i=0; i < v.size(); ++i) {
        std::cout << "--------- " << i << " \n";
        std::cout << v[i] << std::endl;
    }
}

int main() {
    std::cout << "test the rectangle class" << std::endl;
    Rectangle rec(1, 2, 0, 0, 0.2);
    // std::cout << rec.area() << std::endl;
    std::cout << rotation(1.6) << std::endl;
    std::cout << "---------------------------" << std::endl;
    printv(rec.get_coordinates());
    std::cout << "---------------------------" << std::endl;
    printv(rec.get_normals());
    std::cout << "---------------------------" << std::endl;
    point p_test(1, 0);
    printv(rec.get_projection(p_test));
    std::cout << "---------------------------" << std::endl;
    Rectangle rec2(1, 2, 3, 0, 0.2);
    Rectangle rec3(1, 2, 0, 0, 0.2);
    std::cout << rec.in_colission(rec2) << std::endl;
    std::cout << rec.in_colission(rec3) << std::endl;
}