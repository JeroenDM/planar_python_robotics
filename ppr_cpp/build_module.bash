swig -c++ -python rectangle.i
g++ -fPIC -c rectangle.cxx rectangle_wrap.cxx -I /home/jeroen/anaconda3/include/python3.6m -I /home/jeroen/Eigen334/
g++ -shared rectangle.o rectangle_wrap.o -o _rectangle.so