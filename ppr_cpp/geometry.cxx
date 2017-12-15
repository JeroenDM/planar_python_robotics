#include "geometry.h"
#include <Eigen/Dense>
#include <math.h>

rmatrix rotation(float angle) {
    rmatrix R;
    R << cos(angle), -sin(angle),
         sin(angle),  cos(angle);
    return R;
}

Rectangle::Rectangle(float x, float y, float px, float py, float a) {
  width = x;
  height = y;
  pos_x = px;
  pos_y = py;
  R = rotation(a);
}

float Rectangle::area() {
    return width*height;
}

std::vector<point> Rectangle::get_coordinates() {
    std::vector<point> points;
    points.push_back(point(pos_x, pos_y));
    points.push_back(R * point(width, 0)      + points[0]);
    points.push_back(R * point(width, height) + points[0]);
    points.push_back(R * point(0, height)     + points[0]);
    return points;
}