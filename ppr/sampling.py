#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module for sampling based motion planning for path following.
"""

import numpy as np
from .cpp.graph import Graph

def cart_to_joint(robot, traj_points, check_collision = False, scene=None):
    """ Convert a path to joint space by descretising and ik.
    
    Every trajectory point in the path is descretised, then for all these
    poses the inverse kinematics are solved.
    
    Parameters
    ----------
    robot : ppr.robot.Robot
    traj_points : list of ppr.path.TrajectoryPt
    check_collision : bool
        If True, a joint solution is only accepted if it does not collide
        with the objects in the scene. (default false)
        Self collision is not checked but assumed to be ensured by the joint
        limits.
    scene : list of ppr.geometry.Rectangle
        A list of objects with which the robot could collide.
    
    Returns
    -------
    list of numpy.ndarray of floats
        A list of arrays with shape (M, ndof) representing possible joint
        positions for every trajectory point.
        The arrays in this list could be very big!
    """
    # input validation
    if check_collision:
        if scene == None:
            raise ValueError("scene is needed for collision checking")
    
    # get discrete version of trajectory points
    cart_traj = []
    for pt in traj_points:
        cart_traj.append(pt.discretise())

    # solve inverse kinematics for every samples traj point
    # I could add some print statements to have info on unreachable points
    joint_traj = []
    for cart_vec in cart_traj:
        qi = []
        for cart_pt in cart_vec:
            sol = robot.ik(cart_pt)
            if sol['success']:
                for qsol in sol['q']:
                    if check_collision:
                        if not robot.check_collision(qsol, scene):
                            qi.append(qsol)
                    else:
                        qi.append(qsol)
        joint_traj.append(np.array(qi))
    return joint_traj

def get_shortest_path(Q, method='bfs'):
    """ Wrapper function to select the shortest path method
    """
    if method == 'bfs':
        return _get_shortest_path_bfs(Q)
    else:
        raise NotImplementedError("The method " + method + " is not implented yet.")

def _get_shortest_path_bfs(Q):
    """ Calculate the shortest path from joint space data
    
    When the path with trajectory points is converted to joint space,
    this data can be used to construct a graph and look for the shortest path.
    The current distance metrix is the l1-norm of joint position difference
    between two points.
    
    I still have to implement maximum joint movement and acceleration limits.
    So technically this will always find a shortest path for now.
    
    Parameters
    ----------
    Q : list of nympy.ndarrays of float
        A list with the possible joint positions for every trajectory point
        along a path.
    
    Returns
    -------
    dict
        A dictionary with a key 'success' to indicate whether a path was found.
        If success is True, then the key 'path' contains a list with the joint
        position for every trajectory point that gives the shortest path.
    
    Notes
    -----
    I have a problem with swig type conversions. Therefore the type of the
    input data is checked and changed from float64 to float32.
    """
    Q = _check_dtype(Q)

    n_path = len(Q)
    # initialize graph
    g = Graph()
    for c in Q:
        if len(c) == 0:
            # one of the trajectory points is not reachable
            return {'success': False}
        g.add_data_column(c)
    g.init_dijkstra()

    # run shortest path algorithm
    #g.run_dijkstra()
    g.run_bfs()

    # print result
    # g.print_graph()
    g.print_path()

    # get joint values for the shortest path
    p_i = g.get_path(n_path)
    print(p_i)
    cost = g.get_path_cost()

    if p_i[0] == -1:
        return {'success': False}
    else:
        res = []
        for k, i in zip(range(n_path), p_i):
            # TODO ugly all the "unsave" typecasting
            qki = Q[k][i].astype('float64')
            res.append(qki)
        
        return {'success': True, 'path': res, 'length': cost}

def _check_dtype(Q):
    """ Change type if necessary to float32
    
    Due to an unresolved issue with swig and numpy, I have to convert the type.
    
    Parameters
    ----------
    Q : list of nympy.ndarrays of float
        A list with the possible joint positions for every trajectory point
        along a path.
    
    Returns
    -------
    list of nympy.ndarrays of float32
    """
    if Q[0].dtype != 'float32':
        print("converting type of Q")
        for i in range(len(Q)):
            Q[i] = Q[i].astype('float32')
    
    return Q