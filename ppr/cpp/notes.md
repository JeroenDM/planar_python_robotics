
To inlude eigen when compiling for swig:

`g++ -fPIC -c rectangle.cxx rectangle_wrap.cxx -I /home/jeroen/anaconda3/include/python3.6m -I /home/jeroen/Eigen334/`

## Testing in c++

Compile rectangle.cxx as above.
Then compile the test function also giving the object file rectangle.o
`g++ test_rectangle.cxx rectangle.o`