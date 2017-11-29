#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 14:56:47 2017

@author: jeroen
"""

import numpy as np

# should find a better fix, but it finally works !
# the name == robot occurs when importing this file
# from the name == main section of another module in this package
if __name__ == "__main__" or __name__ == "robot":
    from geometry import Rectangle, rotation
else:
    from .geometry import Rectangle, rotation


class Robot:
    """ serial links robot in 2D
    
    assume base at (0, 0)
    assume last link is end effector
    """ 
    
    def __init__(self, joint_type, j_offset, j_angle):
        """ Robot with a rectangle for every link """
        
        # all input lists must have the same length
        self.ndof = len(joint_type)
        self.jt = joint_type
        self.d = j_offset
        self.a = j_angle
        
        # default collision shapes
        self.lx = [0] * self.ndof
        self.ly = [0] * self.ndof
        self.lw = [0.05] * self.ndof
        self.ll = j_offset
        self.adapt_ll = True
        
        # default base pose
        self.base = [0, 0, 0]
        
        # parameters that have to be set explicitly if needed
        self.q_up   = None
        self.q_lo   = None
        self.dq_up  = None
        self.dq_lo  = None
        self.ddq_up = None
        self.ddq_lo = None
        self.m      = None
        self.c      = None # position of center of gravity along link
        self.I      = None # mass moment of inertia around cg

    def set_link_collision_shape(self, lx, ly, lw, ll):
        """ set collision origin and width and length for all links """
        pass
    
    def set_link_inertia(self, mass, cg_position, Icg):
        self.m = mass
        self.c = cg_position
        self.I = Icg

    def set_joint_limits(self, upper_limit, lower_limit=None):
        if lower_limit == None:
            self.q_up =  np.array(upper_limit)
            self.q_lo = -np.array(upper_limit)
        else:
            self.q_up = np.array(upper_limit)
            self.q_lo = np.array(lower_limit)
    
    def set_joint_speed_limits(self, upper_limit, lower_limit=None):
        if lower_limit == None:
            self.dq_up =  np.array(upper_limit)
            self.dq_lo = -np.array(upper_limit)
        else:
            self.dq_up = np.array(upper_limit)
            self.dq_lo = np.array(lower_limit)

    def set_joint_acceleration_limits(self, upper_limit, lower_limit=None):
        if lower_limit == None:
            self.ddq_up =  np.array(upper_limit)
            self.ddq_lo = -np.array(upper_limit)
        else:
            self.ddq_up = np.array(upper_limit)
            self.ddq_lo = np.array(lower_limit)
    
    def set_base_pose(self, pose):
        self.base = pose

    def check_joint_limits(self, q):
        if np.all(self.q_up == None):
            raise RuntimeError("Joint limits not set. Use 'set_joint_limits'.")
        else:
            for i, qi in enumerate(q):
                if qi <= self.q_lo[i] or qi >= self.q_up[i]:
                    return False
            return True
    
    def check_collision(self, q, other_rectangles):
        """ Check for collision between the robot other_rectangles """
        
        for recti in self.get_rectangles(q):
            for rectj in other_rectangles:
                if recti.in_collision(rectj):
                    return True # exit if a collision is found
        return False
    
    def plot_kinematics(self, axes_handle, q, *arg, **karg):
        p = self.fk_all_links(q)
        axes_handle.plot(p[:, 0], p[:, 1], *arg, **karg)
    
    def plot_path_kinematics(self, axes_handle, qp):
        alpha = np.linspace(1, 0.2, len(qp))
        for i, qi in enumerate(qp):
            # color = (0.06, 0.59, 0.13) (green)
            self.plot_kinematics(axes_handle, qi, color=(0.1, 0.2, 0.5, alpha[i]))

    def plot(self, axes_handle, q, *arg, **karg):
        """ Plot the robot with joint position q """
        for recti in self.get_rectangles(q):
            recti.plot(axes_handle, *arg, **karg)

    def plot_path(self, axes_handle, qp):
        """ Plot a list of joint positions """
        
        # draw robot more transparant to the end of the path
        alpha = np.linspace(1, 0.2, len(qp))
        
        for i, qi in enumerate(qp):
            for rect in self.get_rectangles(qi):
                rect.plot(axes_handle, color=(0.1, 0.2, 0.5, alpha[i]))

    def get_link_shape(self, i, xi, yi, phii, qi=None):
        """ Get a rectangle of a specific robot with pose [xi, yi, phii] """
        if self.adapt_ll and (self.jt[i] == 'p') and (qi != None):
            return Rectangle(xi, yi, qi, self.lw[i], phii)
        else:
            return Rectangle(xi, yi, self.ll[i], self.lw[i], phii)
    
    def get_rectangles(self, q):
        """ Get a list with rectangles for all robot links in position q """
        p = self.fk_all_links(q)
        rect = []
        for i in range(self.ndof):
            rect.append(self.get_link_shape(i, p[i, 0], p[i, 1], p[i+1, 2], q[i]))
        return rect

    def fk_all_links(self, q):
        """ Calculate position of all links, and the end effector """
        
        pos = np.zeros((self.ndof+1, 3))
        for i in range(self.ndof):
            if self.jt[i] == 'r':
                pos[i+1, 2] = pos[i, 2] + q[i]  # absolute orientation link i
                pos[i+1, 0] = pos[i, 0] + self.d[i] * np.cos(pos[i+1, 2])
                pos[i+1, 1] = pos[i, 1] + self.d[i] * np.sin(pos[i+1, 2])
            elif self.jt[i] == 'p':
                pos[i+1, 2] = pos[i, 2] + self.a[i]  # absolute orientation link i
                pos[i+1, 0] = pos[i, 0] + q[i] * np.cos(pos[i+1, 2])
                pos[i+1, 1] = pos[i, 1] + q[i] * np.sin(pos[i+1, 2])
            else:
                raise ValueError("wrong joint type: " + self.jt[i])
        
        # transform from base to world
        if self.base[2] == 0:
            # only translation
            pos[:, :2] = pos[:, :2] + np.array(self.base)[:2]
        else:
            pos[:, 2] += self.base[2]
            R = rotation(self.base[2])
            pos[:, :2] = np.dot(R, pos[:, :2].T).T + self.base[:2]
        return pos
    
    def fk(self, q):
        """ Forward kinematics """
        return self.fk_all_links(q)[-1]
    
    def euler_newton(self, q, dq, ddq):
        """ Euler-Netwon recursive algorithm from book Sicilliano """
        ndof = self.ndof
        
        # initialize vectors for solution (base at index 0)
        # i = 0 -> base
        # i = ndof+1 - > last link
        # assume base is not moving TODO
        W  = np.zeros(ndof + 1) # link angular velocity
        dW = np.zeros(ndof + 1) # link angular acceleration
        A  = np.zeros((ndof + 1, 2)) # frame acceleration
        Ac = np.zeros((ndof + 1, 2)) # link cg acceleration
        
        # forward recursion for speed and acceleration
        for k in range(1, ndof+1):
            i = k-1 # link index in robot parameters
            W[k], dW[k], A[k], Ac[k] = self._fwr(i, q[i], dq[i], ddq[i],
                                                 W[k-1], dW[k-1], A[k-1])
        
        # initialize some more vectors (OTHER INDEX DEFINITION than W, dW, ...)
        # i = ndof+1 -> end effector
        # i = 0 - > first link
        # assume no end effector force or torque
        F = np.zeros((ndof + 1, 2))
        M = np.zeros(ndof + 1)
        t = np.zeros(ndof) # joint force or torque
        
        # backward recursion for forces and torques
        print("backward")
        for k in np.flip(np.arange(ndof), 0):
            print(k)
            i = k # link index in robot parameters
            F[k], M[k], t[k] = self._bwr(q[i], dq[i], ddq[i],
                                         F[k+1], M[k+1],
                                         W[k+1], dW[k+1],
                                         A[k+1], Ac[k+1])
        return t
    
    def _fwr(self, i, q, dq, ddq, w, dw, a):
        """ speed of next frame based on speed previous frame """
        from numpy import sin as s
        from numpy import cos as c
        # first angular velocity and frame i acceleration
        if self.jt[i] == 'p':
            ai = self.a[i]
            rx = q * c(ai)
            ry = q * s(ai)
            
            wi = w
            aix = ddq * c(ai) + 2 * dq * w * s(ai) + dw * ry - w**2 * rx 
            aiy = ddq * s(ai) - 2 * dq * w * c(ai) - dw * rx - w**2 * ry
            
        elif self.jt[i] == 'r':
            rx = self.d[i] * c(q)
            ry = self.d[i] * s(q)
            
            wi = w + dq
            aix =  dw * ry - w**2 * rx 
            aiy = -dw * rx - w**2 * ry
            
        else:
            raise ValueError("wrong joint type: " + self.jt[i])
        ai  = np.array([aix, aiy])
        
        # cg acceleration ang angular acceleration
        dwi = 0
        aci = np.zeros(2)
        return wi, dwi, ai, aci
    
    def _bwr(self, q, dq, ddq, f, m, w, dw, a, ac):
        fi = np.zeros(2)
        mi = 0
        ti = 0
        return fi, mi, ti
    
    def _fw_prop(self, ib, va, aa, wa, dwa, qb, dqb, ddqb):
        """ Forward propagation of speed and velocity from link i to i+1 """
        if self.jt[ib] == 'r':
            db = self.d[ib]
            vrel = [0, (wa + dqb) * db]
            arel = [-(wa + dqb)**2 * db, (dwa + ddqb)*db]
            wb = wa + dqb
            dwb = dwa + ddqb
        elif self.jt[ib] == 'p':
            vrel = [dqb, wa * db]
            arel = [-wa**2 * db + ddqb, dwa*db + 2*wa*dqb]
            wb = wa
            dwb = dwa
        else:
            raise ValueError("wrong joint type: " + self.jt[ib])
        
        R = self._R_link(ib, qb)
        vb = np.dot(R, va) + np.array(vrel)
        ab = np.dot(R, aa) + np.array(arel)
        return vb, ab, wb, dwb
    
    def _bw_prop(self, ia, Fb, Mb, aca, dwa, qb):
        # transforme Fb to frame of link a (given in frame b)
        R = self._R_link(ia+1, qb).T
        Fbt = np.dot(R, Fb)
        
        # calculate Fa and Ma
        Fa = Fb + self.m[ia] * Fbt
        Ma = Mb + self.cg[ia] * Fa[1]
        Ma += (self.d[ia] - self.cg[ia]) * Fb[1]
        Ma += self.I[ia] * dwa
        
        # calculate joint torque or force
        if self.jt[ia] == 'r':
            ja = Ma
        elif self.jt[ia] == 'p':
            ja = Fa[0]
        else:
            raise ValueError("wrong joint type: " + self.jt[ia])
        
        return Fa, Ma, ja
    
    def _R_link(self, i, qi):
        """ rotation matrix of link i relative to link i-1 """
        if self.jt[i] == 'r':
            a = qi
        elif self.jt[i] == 'p':
            a = self.a[i]
        else:
            raise ValueError("wrong joint type: " + self.jt[i])
        
        ca = np.cos(a)
        sa = np.sin(a)
        R = np.array([[ca, -sa], [sa, ca]])
        return R

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    print("-----test robot.py-----")
    print("--------------")
    fig = plt.figure()
    ax = fig.gca()
    plt.axis('equal')
    plt.axis([-1, 3, -1, 3])
    
    qr = np.linspace(0, 1, 10)
    qt = np.array([qr, qr, qr]).T
    
    r1 = Robot(['r', 'r', 'r'], [1, 1, 0.5], [0, 0, 0])
    r1.set_base_pose([0.3, 0.6, 0])
    r1.plot_path_kinematics(ax, qt)
    
    r1.set_base_pose([0.3, 0.3, -0.5])
    r1.plot_path_kinematics(ax, qt)
    
    print("speed and acc propatation")
    from numpy.random import rand
    v = rand(2)
    a = rand(2)
    v2, a2, w2, dw2 = r1._fw_prop(1, v, a, 0.5, 0.5, 0.2, 0.2, 0.2)
    print(v2)
    print(a2)
    print(w2, dw2)
    
#    qt = [0, 0.1, 0.2]
#    dqt = [0, 0.1, 0.2]
#    ddqt = [0, 0.1, 0.2]
#    print(r1.euler_newton(qt, dqt, ddqt))
#    
#    print("test joint limits. Expect False")
#    r1.set_joint_limits([1, 1, 3])
#    r1.set_joint_speed_limits([1, 2, 3])
#    r1.set_joint_acceleration_limits([1, 2, 3])
#    print(r1.check_joint_limits([1, 2, 1]))