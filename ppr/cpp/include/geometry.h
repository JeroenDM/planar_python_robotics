#ifndef _GEOMETRY_H_
#define _GEOMETRY_H_

#include <vector>
#include <Eigen/Dense>

typedef Eigen::Matrix2f rmatrix;
typedef Eigen::Vector2f point;

const double PI = 3.141592653589793238462643383279502884L;

rmatrix rotation(float);

class Rectangle {
    float width, height, pos_x, pos_y;
    rmatrix R;
    std::vector<point> p;
    float tolerance;

    std::vector<point> _get_vertices();
    std::vector<point> _get_normals();
    std::vector<float> get_projection(point direction);

  public:
    Rectangle(float, float, float, float, float);
    void set_tolerance(float new_tolerance);
    bool is_in_collision(Rectangle other);
    void get_vertices(double mat[4][2]);
    void get_normals(double mat[4][2]);
};

#endif