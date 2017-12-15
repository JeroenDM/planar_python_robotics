#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Matrix2f rmatrix;
typedef Eigen::Vector2f point;

rmatrix rotation(float);

class Rectangle {
    float width, height, pos_x, pos_y;
    rmatrix R;
  public:
    Rectangle(float, float, float, float, float);
    float area();
    std::vector<point> get_coordinates();
};

#endif