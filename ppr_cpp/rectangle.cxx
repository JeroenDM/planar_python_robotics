#include "rectangle.h"
#include <Eigen/Dense>

void Rectangle::set_values (float x, float y) {
  width = x;
  height = y;
}

float Rectangle::area() {
    return width*height;
}

void Rectangle::eigen_stuff() {
    using Eigen::MatrixXd;
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
}