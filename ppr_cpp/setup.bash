swig -c++ -python geometry.i
g++ -std=c++11 -O2 -fPIC -c geometry.cxx geometry_wrap.cxx -I /home/jeroen/anaconda3/include/python3.6m -I /home/jeroen/Eigen3/
g++ -std=c++11 -O2 -shared geometry.o geometry_wrap.o -o _geometry.so

swig -c++ -python graph.i
g++ -std=c++11 -O2 -fPIC -c graph.cxx graph_wrap.cxx -I /home/jeroen/anaconda3/include/python3.6m
g++ -std=c++11 -O2 -shared graph.o graph_wrap.o -o _graph.so