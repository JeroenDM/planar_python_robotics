#include <iostream>
#include "rectangle.h"

int main() {
    std::cout << "test the rectangle class" << std::endl;
    Rectangle rec;
    rec.set_values(4, 5);
    std::cout << rec.area() << std::endl;
}