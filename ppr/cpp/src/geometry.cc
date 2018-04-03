#include "geometry.h"

#include <Eigen/Dense>
#include <iostream>
#include <math.h>

RotationMatrix create_rotation_matrix(double angle) {
    RotationMatrix R;
    R << cos(angle), -sin(angle),
         sin(angle),  cos(angle);
    return R;
}

Rectangle::Rectangle(double px, double py, double dx, double dy, double a) {
  // TODO turn around other point than left bottom corner
  width_ = dx;
  height_ = dy;
  pos_x_ = px;
  pos_y_ = py;
  rotation_matrix_ = create_rotation_matrix(a);
  vertices_ = _get_vertices();
  tolerance_ = 1e-6;
}

void Rectangle::set_tolerance(double new_tolerance) {
    tolerance_ = new_tolerance;
}

std::vector<Vector2D> Rectangle::_get_vertices() {
    // Vector2Ds are ordered counterclockwise
    std::vector<Vector2D> points;
    points.push_back(Vector2D(pos_x_, pos_y_));
    points.push_back(rotation_matrix_ * Vector2D(width_, 0)      + points[0]);
    points.push_back(rotation_matrix_ * Vector2D(width_, height_) + points[0]);
    points.push_back(rotation_matrix_ * Vector2D(0, height_)     + points[0]);
    return points;
}

std::vector<Vector2D> Rectangle::_get_normals() {
    std::vector<Vector2D> normals;
    // Outside pointing normal on the first side
    // (counterclockwise)
    Vector2D n0;
    n0[0] =   0.0;
    n0[1] =   -1.0;
    normals.push_back(rotation_matrix_ * n0);
    // Rotate this n0 3 times for the other normals
    RotationMatrix Rtemp = create_rotation_matrix(PI/2);
    normals.push_back(Rtemp * normals[0]);
    normals.push_back(Rtemp * normals[1]);
    normals.push_back(Rtemp * normals[2]);
    return normals;
}

std::vector<double> Rectangle::get_projection(Vector2D direction) {
    double angle = -atan2(direction[1], direction[0]);
    RotationMatrix Rtemp = create_rotation_matrix(angle);
    std::vector<double> proj;
    for (int i=0; i<4; ++i) {
        Vector2D ptemp = Rtemp * vertices_[i];
        proj.push_back(ptemp[0]);
    }
    return proj;
}

bool Rectangle::is_in_collision(Rectangle other) {
    std::vector<Vector2D> n1 = _get_normals();
    std::vector<Vector2D> n2 = other._get_normals();
    // concatenate n1 and n2, save in n1
    n1.insert(n1.end(), n2.begin(), n2.end());
    bool col = true;
    int i = 0;
    while (col and i < 8) {
        std::vector<double> proj1 = get_projection(n1[i]);
        std::vector<double> proj2 = other.get_projection(n1[i]);
        double max1, max2, min1, min2;
        // calculate min and max
        max1 = *std::max_element(proj1.begin(), proj1.end());
        max2 = *std::max_element(proj2.begin(), proj2.end());
        min1 = *std::min_element(proj1.begin(), proj1.end());
        min2 = *std::min_element(proj2.begin(), proj2.end());
        if ((max1 + tolerance_ < min2) or (min1 > max2 + tolerance_)) {
            col = false;
        }
        ++i;
    }
    return col;
}

void Rectangle::get_vertices(double mat[4][2]) {
    std::vector<Vector2D> temp = vertices_;
    for (int i=0; i < temp.size(); ++i) {
        mat[i][0] = temp[i][0];
        mat[i][1] = temp[i][1];
    }
}

void Rectangle::get_normals(double mat[4][2]) {
    std::vector<Vector2D> temp = _get_normals();
    for (int i=0; i < temp.size(); ++i) {
        mat[i][0] = temp[i][0];
        mat[i][1] = temp[i][1];
    }
}