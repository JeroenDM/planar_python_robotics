
import numpy as np
import graph as gr

m = np.array([[0, 0, 0],
              [2, 1, 0.5],
              [3, 1.5, 1]])
print(m)

gr.input_matrix(m)

g = gr.Graph()
g.add_data_column(m[0])
g.add_data_column(m[1])
g.add_data_column(m[2])
g.print_graph_data()

g.init_dijkstra()
g.print_graph()

g.run_dijkstra()
g.print_path()