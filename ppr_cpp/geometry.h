#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Matrix2f rmatrix;
typedef Eigen::Vector2f point;

const double PI = 3.14159265358979323846;

rmatrix rotation(float);

class Rectangle {
    float width, height, pos_x, pos_y;
    rmatrix R;
    std::vector<point> p;
  public:
    Rectangle(float, float, float, float, float);
    float area();
    std::vector<point> get_coordinates();
    std::vector<point> get_normals();
};

#endif