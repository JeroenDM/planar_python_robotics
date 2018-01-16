
from ppr_cpp.graph_cpp import get_shortest_path
import numpy as np

m = np.array([[0, 0, 0],
              [2, 1, 0.5],
              [3, 1.5, 1]])
# print(m)

mm = []
for i in range(3):
    temp = []
    for j in range(3):
        temp.append( np.array([m[i, j], 0, 0, 0]) )
    mm.append(np.array(temp))

p = get_shortest_path(mm)
print(p)

