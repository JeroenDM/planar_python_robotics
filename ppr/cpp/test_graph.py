
import numpy as np
import graph

# m = np.array([[7, 0, 0],
#               [7.5, 1, 0.5],
#               [8, 1.5, 1]])
m = np.array([[0, 7, 0],
              [1, 7.5, 0.5],
              [1.5, 8, 1]])
# print(m)

mm = []
for i in range(3):
    temp = []
    for j in range(3):
        temp.append( np.array([m[i, j], 0, 0, 0], dtype='float32') )
    mm.append(np.array(temp))

# print(mm[0].shape)
# gr.input_matrix(mm[1])

print(mm)

g = graph.Graph()
g.add_data_column(mm[0])
g.add_data_column(mm[1])
g.add_data_column(mm[2])
# g.print_graph_data()

g.init_dijkstra()
# g.print_graph()

g.run_dijkstra()
g.print_path()

p = g.get_path(len(mm))
print(p)