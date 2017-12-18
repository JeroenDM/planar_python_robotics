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
    float tolerance;
  public:
    Rectangle(float, float, float, float, float);
    void set_tolerance(float new_tolerance);
    std::vector<point> get_coordinates();
    std::vector<point> get_normals();
    std::vector<float> get_projection(point direction);
    bool in_collision(Rectangle other);
    void get_plot_points(double mat[4][2]);
};

#endif