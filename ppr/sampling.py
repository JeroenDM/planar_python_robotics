#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module for sampling based motion planning for path following.
"""

import numpy as np
#import multiprocessing as mp
import pathos.multiprocessing as mp
from .cpp.graph import Graph
from .path import TolerancedNumber, TrajectoryPt

class SolutionPoint:
    """ class to save intermediate solution info for trajectory point
    """
    def __init__(self, tp):
        self.tp_init = tp
        self.tp_current = tp
        self.q_best = []
        self.jl = []
    
    def get_joint_solutions(self, robot, check_collision = False, scene=None):
        """ Convert a cartesian trajectory point to joint space """
        
        # input validation
        if check_collision:
            if scene == None:
                raise ValueError("scene is needed for collision checking")
        
        # use different joint limits for redundant joints
        if robot.ndof > 3:
            # save origanal joint limits
            orig_jl = robot.jl
            robot.jl = self.jl
        
        tp_discrete = self.tp_current.discretise()
        joint_solutions = []
        for cart_pt in tp_discrete:
            sol = robot.ik(cart_pt)
            if sol['success']:
                for qsol in sol['q']:
                    if check_collision:
                        if not robot.check_collision(qsol, scene):
                            joint_solutions.append(qsol)
                    else:
                        joint_solutions.append(qsol)
        
        if robot.ndof > 3:
            # reset original joint_limits
            robot.jl = orig_jl 
        
        return np.array(joint_solutions)
    
    def get_new_bounds(self, l, u, m, red=4):
        """ create new interval smaller than the old one (l, u)
        reduced in length by a factor red.
        m is the value around wich the new interval should be centered
        the new interval may not go outside the old bounds
        """
        delta = abs(u - l) / red
        l_new = max(m - delta, l)
        u_new = min(m + delta, u)
        return l_new, u_new
    
    def resample_trajectory_point(self, robot, *arg, **kwarg):
        """ create a new trajectory point with smaller bounds,
        but same sample number
        use the value from the forward kinematics pfk as the center
        of the new interval
        """
        pfk = robot.fk(self.q_best)
        p_new = []
        for i, val in enumerate(self.tp_current.p):
            if self.tp_current.hasTolerance[i]:
                # check for rounding errors on pfk
                if pfk[i] < val.l:
                    pfk[i] = val.l
                if pfk[i] > val.u:
                    pfk[i] = val.u
                # now calculate new bounds with corrected pkf
                l, u = self.get_new_bounds(val.l, val.u, pfk[i], *arg, **kwarg)
                val_new = TolerancedNumber(pfk[i], l, u, samples=val.s)
            else:
                val_new = val
            p_new.append(val_new)
        self.tp_current = TrajectoryPt(p_new)

#def calculate_joint_solutions(a):
#    return a[0].get_joint_solutions(a[1], check_collision=True, scene=a[2])            
        
def iterative_bfs(robot, path, scene, tol=0.001, red=10, max_iter=10):
    """ Iterative graph construction and search """
    pool = mp.Pool(processes=4)
    sol_pts = [SolutionPoint(tp) for tp in path]
    if robot.ndof > 3:
        for i in range(len(sol_pts)):
            sol_pts[i].jl = robot.jl
    costs = []
    prev_cost = np.inf
    success = False
    for i in range(max_iter):
        # parallel joint solution computation
        def calculate_joint_solutions(point):
            return point.get_joint_solutions(robot, check_collision=True, scene=scene)
        #arg = [[sp, robot, scene] for sp in sol_pts]
        path_js = list(pool.map(calculate_joint_solutions, sol_pts))
        #path_js = list(map(calculate_joint_solutions, sol_pts))
        #path_js = [sp.get_joint_solutions(robot, check_collision=True, scene=scene) for sp in sol_pts]
        sol = get_shortest_path(path_js)
        if sol['success']:
            costs.append(sol['length'])
            if abs(prev_cost - sol['length']) < tol:
                success = True
                break
            else:
                prev_cost = sol['length']
            for i in range(len(sol_pts)):
                sol_pts[i].q_best = sol['path'][i]
                sol_pts[i].resample_trajectory_point(robot, red=red)
                # if redundant robot, update joint limits
                # TODO hard coded that the first joints are the redundant ones
                if robot.ndof > 3:
                    old_joint_limits = sol_pts[i].jl
                    new_joint_limits = []
                    for j in range(len(robot.ik_samples)):
                        jl = old_joint_limits[j]
                        qj = sol['path'][i][j]
                        # check for rounding errors on qj
                        if qj < jl[0]:
                            qj = jl[0]
                        if qj > jl[1]:
                            qj = jl[1]
                        l, u = sol_pts[i].get_new_bounds(jl[0], jl[1], qj, red=red)
                        new_joint_limits.append((l, u ))
                    sol_pts[i].jl = new_joint_limits
                        
        else:
            return {'success': False, 'info': 'stuck when looking for path'}
    
    if success:
        return {'success': True,
                'path': sol['path'],
                'length': sol['length'],
                'length_all_iterations': costs}
    else:
        return {'success': False,
                'path': sol['path'],
                'length': sol['length'],
                'length_all_iterations': costs,
                'info': 'max_iterations_reached'}


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

def get_shortest_path(Q, method='bfs', path = None, scene = None):
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