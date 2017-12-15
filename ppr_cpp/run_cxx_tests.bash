g++ -c geometry.cxx -I /home/jeroen/Eigen3/
g++ test_geometry.cxx geometry.o -o test.out -I /home/jeroen/Eigen3/
./test.out