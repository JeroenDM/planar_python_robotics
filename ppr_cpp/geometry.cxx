#include "geometry.h"
#include <Eigen/Dense>
#include <math.h>
#include <iostream>

rmatrix rotation(float angle) {
    rmatrix R;
    R << cos(angle), -sin(angle),
         sin(angle),  cos(angle);
    return R;
}

void abel(int len1, double* ivec) {
    Eigen::VectorXd v(len1);
    for (int i=0; i<len1; ++i) {
        v[i] = ivec[i];
    }
    std::cout << "Abel received a vector!\n";
    ivec[0] = 99.0;
    std::cout << v << std::endl;
}

void range(int *rangevec, int n) {
    int i;
    for (i=0; i<n; ++i) {
        rangevec[i] = i;
    }
}

void jane(double mat[2][2]) {
    std::cout << "This is jane" << std::endl;
    mat[0][0] = 99.0;
    mat[0][1] = 2.0;
    mat[1][0] = 3.0;
    mat[1][1] = 4.0;
}

void kane(double* data, int nrows, int ncols) {
    Eigen::MatrixXd m(nrows, ncols);
    std::cout << "Kane received a matrix!\n";
    for (int i=0; i<nrows; ++i) {
        for (int j=0; j<ncols; ++j) {
            int index = j + (nrows+1)*i;
            m(i, j) = data[index];
        }
    }
    std::cout << m << std::endl;
}

Rectangle::Rectangle(float px, float py, float dx, float dy, float a) {
  width = dx;
  height = dy;
  pos_x = px;
  pos_y = py;
  R = rotation(a);
  p = get_coordinates();
  tolerance = 1e-6;
}

void Rectangle::set_tolerance(float new_tolerance) {
    tolerance = new_tolerance;
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
    normals.push_back(Rtemp * normals[1]);
    normals.push_back(Rtemp * normals[2]);
    return normals;
}

std::vector<float> Rectangle::get_projection(point direction) {
    float angle = -atan2(direction[1], direction[0]);
    rmatrix Rtemp = rotation(angle);
    std::vector<float> proj;
    for (int i=0; i<4; ++i) {
        point ptemp = Rtemp * p[i];
        proj.push_back(ptemp[0]);
    }
    return proj;
}

bool Rectangle::in_collision(Rectangle other) {
    std::vector<point> n1 = get_normals();
    std::vector<point> n2 = other.get_normals();
    // concatenate n1 and n2, save in n1
    n1.insert(n1.end(), n2.begin(), n2.end());
    bool col = true;
    int i = 0;
    while (col and i < 8) {
        std::vector<float> proj1 = get_projection(n1[i]);
        std::vector<float> proj2 = other.get_projection(n1[i]);
        float max1, max2, min1, min2;
        // calculate min and max
        max1 = *std::max_element(proj1.begin(), proj1.end());
        max2 = *std::max_element(proj2.begin(), proj2.end());
        min1 = *std::min_element(proj1.begin(), proj1.end());
        min2 = *std::min_element(proj2.begin(), proj2.end());
        if ((max1 + tolerance < min2) or (min1 > max2 + tolerance)) {
            col = false;
        }
        ++i;
    }
    return col;
}

void Rectangle::get_plot_points(double mat[4][2]) {
    std::vector<point> temp = get_coordinates();
    for (int i=0; i < temp.size(); ++i) {
        mat[i][0] = temp[i][0];
        mat[i][1] = temp[i][1];
    }
}