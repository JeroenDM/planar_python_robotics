g++ -std=c++11 -g -c src/graph.cc -I include
g++ -std=c++11 -g -c tests/test_graph.cc -I include -o tests/test_graph.o
g++ -o tests/run_tests tests/test_graph.o graph.o
./tests/run_tests