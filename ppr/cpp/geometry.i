%module geometry

%{
    #define SWIG_FILE_WITH_INIT
    #include "include/geometry.h"
%}

%include "numpy.i"

%init %{
  import_array();
%}

// apply numpy typemaps for input stuff
//%apply (int DIM1, double* IN_ARRAY1) {(int len1, double* ivec)}
//%apply (double* IN_ARRAY2, int DIM1 , int DIM2){(double* data, int nrows, int ncols)}

// apply numpy typemaps for output stuff
%apply( double ARGOUT_ARRAY2[ANY][ANY] ) {(double mat[2][2]), (double mat[4][2])}

//%apply(int* ARGOUT_ARRAY1, int DIM1) {(int* rangevec, int n)}
// %apply (double** ARGOUTVIEWM_ARRAY2, int* DIM1, int* DIM2) {(double** data, int* nrows, int* ncols)}

%ignore create_rotation_matrix(float);
%include "include/geometry.h"