swig -c++ -python graph.i
g++ -std=c++11 -O2 -fPIC -c src/graph.cc graph_wrap.cxx -I /home/jeroendm/anaconda3/include/python3.6m -I include
g++ -std=c++11 -O2 -shared graph.o graph_wrap.o -o _graph.so

swig -c++ -python geometry.i
g++ -std=c++11 -O2 -fPIC -c src/geometry.cc geometry_wrap.cxx -I /home/jeroendm/anaconda3/include/python3.6m -I include -I /usr/include/eigen3
g++ -std=c++11 -O2 -shared geometry.o geometry_wrap.o -o _geometry.so
