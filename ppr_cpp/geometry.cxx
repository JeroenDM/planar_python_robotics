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
  p = get_coordinates();
}

float Rectangle::area() {
    return width*height;
}

std::vector<point> Rectangle::get_coordinates() {
    // points are ordered counterclockwise
    std::vector<point> points;
    points.push_back(point(pos_x, pos_y));
    points.push_back(R * point(width, 0)      + points[0]);
    points.push_back(R * point(width, height) + points[0]);
    points.push_back(R * point(0, height)     + points[0]);
    return points;
}

std::vector<point> Rectangle::get_normals() {
    std::vector<point> normals;
    // Outside pointing normal on the first side
    // (counterclockwise)
    point n0;
    n0[0] =   p[1][1] - p[0][1];
    n0[1] =   -(p[1][0] - p[0][0]);
    n0 = n0 / n0.norm();
    normals.push_back(n0);
    // Rotate this n0 3 times for the other normals
    rmatrix Rtemp = rotation(PI/2);
    normals.push_back(Rtemp * normals[0]);
    normals.push_back(Rtemp * normals[0]);
    normals.push_back(Rtemp * normals[0]);
    return normals;
}