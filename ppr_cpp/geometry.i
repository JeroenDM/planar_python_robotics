/* prime.i */
%module geometry
%{
#include "geometry.h"
%}
%ignore rotation(float);
%include "geometry.h"