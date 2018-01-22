
if (__name__ == "ppr.cpp.graph_cpp"):
    import ppr.cpp.graph as graph
else:
    import graph as graph

import numpy as np

def get_shortest_path(Q):
    Q = _check_dtype(Q)

    n_path = len(Q)
    # initialize graph
    g = graph.Graph()
    for c in Q:
        g.add_data_column(c)
    g.init_dijkstra()

    # run shortest path algorithm
    g.run_dijkstra()

    # print result
    # g.print_graph()
    g.print_path()

    # get joint values for the shortest path
    p_i = g.get_path(n_path)
    print(p_i)

    if p_i[0] == -1:
        return {'success': False}
    else:
        res = []
        for k, i in zip(range(n_path), p_i):
            res.append(Q[k][i])
        
        return {'success': True, 'path': res}

def _check_dtype(Q):
    if Q[0].dtype == 'float64':
        print("converting type of Q")
        for i in range(len(Q)):
            Q[i] = Q[i].astype('float32')
    
    return Q
