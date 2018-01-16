%module graph

%{
    #define SWIG_FILE_WITH_INIT
    #include "graph.h"
%}

%include "numpy.i"

%init %{
  import_array();
%}

// apply numpy typemaps for input stuff
%apply (double* IN_ARRAY1, int DIM1) {(double* vec, int n)}
%apply (double* IN_ARRAY2, int DIM1 , int DIM2){(double* mat, int nrows, int ncols)}

%apply ( int* ARGOUT_ARRAY1, int DIM1 ){(int* vec, int n)}

%ignore Node;
%ignore sort_function(Node*, Node*);
%include "graph.h"