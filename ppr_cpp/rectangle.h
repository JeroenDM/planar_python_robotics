#ifndef _RECTANGLE_H_
#define _RECTANGLE_H_

class Rectangle {
    float width, height;
  public:
    void set_values (float,float);
    float area();
    void eigen_stuff();
};

#endif