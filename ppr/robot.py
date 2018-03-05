#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from .geometry import Rectangle, rotation

class Robot:
    """ Base class for a serial manipulator
    
    Attributes
    ----------
    ndof : int
        Number of degrees of freedom, equal to the number of links.
    jt : list of char
        List of joint type for every joint, 'r' or 'p'.
    d : list of float
        Link length for links with revolute joint.
        Initial joint length when link has prismatic joint.
    a : Link angle relative to previous link for prismatic joint.
        Initial joint angle for links with revolute joint.
    lw : list of float
        Link width.
    ll : list of float
        Link length. In case of a prismatic joint, this is de initial length.
    adapt_ll : bool
        When their are prismatic joints, the length of those links can change
        accroding to the given joint values if set to True. (default is True)
    base : numpy.ndarray of float
        Pose of the robot base (x, y, angle), where the first joint is
        located.
    """
    
    def __init__(self, joint_type, j_offset, j_angle):
        """ Create general robot object
        
        Parameters
        ----------
        joint_type : list of char
            Use 'p' for prismatic and 'r' for revolute.
        j_offset : list of float
            Link length. Initial length for a prismatic joint.
        j_angle : list of float
            Link angle. Initial angle for revolute joint.
        """
        
        # all input lists must have the same length
        self.ndof = len(joint_type)
        self.jt = joint_type
        self.d = j_offset
        self.a = j_angle
        
        # default collision shapes
        self.lw = [0.05] * self.ndof
        self.ll = j_offset
        self.adapt_ll = True
        
        # default base pose
        self.base = np.array([0, 0, 0])
    
    def fk(self, q):
        """ Calculate forward kinematics
        
        Uses the fk_all_links function because it's almost the same.
        
        Parameters
        ----------
        q : list or numpy.array of float
            Joint angles
        
        Examples
        --------
        
        Example for a simple 2R robot.
        
        >>> robot1 = Robot(['r', 'r'], [2, 1], [0, 0])
        >>> robot1.fk([np.pi/2, -np.pi/2])
        array([ 1.,  2.,  0.])
        
        """
        return self.fk_all_links(q)[-1]
    
    def fk_all_links(self, q):
        """ Calculate position of all links, and the end effector
        
        The end-effector is currently assumed at the end of the last link.
        Usefull for collision detection and visualizing the robot.
        
        Parameters
        ----------
        q : list or numpy.array of float
            Joint angles
        
        Returns
        -------
        numpy.ndarray
            Array of shape (ndof+1, 3) containing position and orientation
            (x, y, angle) of all links and the end-effector.
        
        Examples
        --------
        
        Example for a simple 2R robot.
        
        >>> robot1 = Robot(['r', 'r'], [2, 1], [0, 0])
        >>> robot1.fk_all_links([np.pi/2, -np.pi/2])
        array([[  0.00000000e+00,   0.00000000e+00,   0.00000000e+00],
               [  1.22464680e-16,   2.00000000e+00,   1.57079633e+00],
               [  1.00000000e+00,   2.00000000e+00,   0.00000000e+00]])
    
        Example for cartesian x-y robot.
        
        >>> robot2 = Robot(['p', 'p'], [1, 1], [0, np.pi/2])
        >>> robot2.fk_all_links([0.3, 0.6])
        array([[ 0.        ,  0.        ,  0.        ],
               [ 0.3       ,  0.        ,  0.        ],
               [ 0.3       ,  0.6       ,  1.57079633]])
        """
        pos = np.zeros((self.ndof+1, 3))
        for i in range(self.ndof):
            if self.jt[i] == 'r':
                # absolute orientation link i
                pos[i+1, 2] = pos[i, 2] + q[i]
                # position
                pos[i+1, 0] = pos[i, 0] + self.d[i] * np.cos(pos[i+1, 2])
                pos[i+1, 1] = pos[i, 1] + self.d[i] * np.sin(pos[i+1, 2])
            elif self.jt[i] == 'p':
                # absolute orientation link i
                pos[i+1, 2] = pos[i, 2] + self.a[i]
                # position
                pos[i+1, 0] = pos[i, 0] + q[i] * np.cos(pos[i+1, 2])
                pos[i+1, 1] = pos[i, 1] + q[i] * np.sin(pos[i+1, 2])
            else:
                msg = "wrong joint type. Must be 'r' or 'p', not: "
                msg += self.jt[i]
                raise ValueError(msg)
        
        # transform from base to world
        if self.base[2] == 0:
            # only translation
            pos[:, :2] = pos[:, :2] + self.base[:2]
        else:
            pos[:, 2] += self.base[2]
            R = rotation(self.base[2])
            pos[:, :2] = np.dot(R, pos[:, :2].T).T + self.base[:2]
        return pos
    
    def get_shapes(self, q):
        """ Get a list with rectangles (shapes in the future?) for all
        robot links in position q
        
        Parameters
        ----------
        q : list or numpy.array of float
            Joint angles
        
        Returns
        -------
        list of ppr.geometry.Rectangle
            List of the link shapes.
        """
        p = self.fk_all_links(q)
        shapes = []
        for i in range(self.ndof):
            shapes.append(self.get_link_shape(i, p[i, 0], p[i, 1], p[i+1, 2], q[i]))
        return shapes

    def get_link_shape(self, i, xi, yi, phii, qi):
        """ Get the rectangle representing link i of the robot
        
        Parameters
        ----------
        i : int
            Link index from 0 to ndof.
        xi : float
            x-position of link origin.
        yi : float
            y-position of link origin.
        phii : float
            Absolute angle of link with x-axis.
        qi : float
            Joint value for the joint of this link, usefull for prismatic
            joints to show variable length.
        
        Returns
        -------
        ppr.geometry.Rectangle
            A rectangle (or in the future a shape) oject representing the
            robot link.
        
        Examples
        --------
        
        Illustrate variable vs fixed prismatic joint length.
        
        >>> robot2 = Robot(['p', 'p'], [2, 1], [0, np.pi/2])
        >>> link0 = robot2.get_link_shape(0, 0, 0, 0, 1.5)
        >>> link0.get_vertices()
        array([[ 0.  ,  0.  ],
               [ 1.5 ,  0.  ],
               [ 1.5 ,  0.05],
               [ 0.  ,  0.05]])
        >>> robot2.adapt_ll = False
        >>> link0 = robot2.get_link_shape(0, 0, 0, 0, 1.5)
        >>> link0.get_vertices()
        array([[ 0.  ,  0.  ],
               [ 2.  ,  0.  ],
               [ 2.  ,  0.05],
               [ 0.  ,  0.05]])
        """
        if self.adapt_ll and (self.jt[i] == 'p'):
            # link length = joint length for prismatic case
            return Rectangle(xi, yi, qi, self.lw[i], phii)
        else:
            # fixed link length
            return Rectangle(xi, yi, self.ll[i], self.lw[i], phii)
    
    def check_collision(self, q, scene):
        """ Check for collision between the robot other_rectangles
        
        Parameters
        ----------
        q : list or numpy.ndarray of float
            Joint position
        scene : list of ppr.geometry.Rectangle
            A list with collision shapes.
        
        Returns
        -------
        bool
            True if one of the robot links collides with one of the collision
            shapes. False otherwise.
        """
        for recti in self.get_shapes(q):
            for rectj in scene:
                if recti.is_in_collision(rectj):
                    return True # exit if a collision is found
        return False
    
    def plot_kinematics(self, axes_handle, q, *arg, **karg):
        """ Plot robot using lines and dots
        
        Ingnore the (rectanlge) collision shapes.
        
        Parameters
        ----------
        axes_handle : matplotlib.axes.Axes
            Axes handle to draw the robot on.
        q : np.array or list of floats
            Joint values
        """
        p = self.fk_all_links(q)
        axes_handle.plot(p[:, 0], p[:, 1], *arg, **karg)
    
    def plot_path_kinematics(self, axes_handle, qp):
        """ Plot robot for more than one joint position using lines and dots
        
        A color gradient is used to from darker to lighter.
        
        Parameters
        ----------
        axes_handle : matplotlib.axes.Axes
            Axes handle to draw all the robots
        qp : np.array of floats
            Array of shape (N, ndof) representing N different joint positions,
            ndof is the number of robot joints.
        """
        alpha = np.linspace(1, 0.2, len(qp))
        for i, qi in enumerate(qp):
            self.plot_kinematics(axes_handle, qi, color=(0.1, 0.2, 0.5, alpha[i]))

    def plot(self, axes_handle, q, *arg, **karg):
        """ Plot robot
        
        Plot robot shape in given joint position.
        
        Parameters
        ----------
        axes_handle : matplotlib.axes.Axes
            Axes handle to draw the robot on.
        q : np.array or list of floats
            Joint values
        """
        for recti in self.get_shapes(q):
            recti.plot(axes_handle, *arg, **karg)

    def plot_path(self, axes_handle, qp):
        """ Plot robot for more than one joint position
        
        A color gradient is used to from darker to lighter.
        
        Parameters
        ----------
        axes_handle : matplotlib.axes.Axes
            Axes handle to draw all the robots
        qp : np.array of floats
            Array of shape (N, ndof) representing N different joint positions,
            ndof is the number of robot joints.
        """
        # draw robot more transparant to the end of the path
        alpha = np.linspace(1, 0.2, len(qp))
        
        for i, qi in enumerate(qp):
            for rect in self.get_shapes(qi):
                rect.plot(axes_handle, color=(0.1, 0.2, 0.5, alpha[i]))
    
    #=============================================================================
    # THE FOLLOWING METHODS ARE NOT TESTED YET AND ALMOST NO DOCUMENTATION
    #=============================================================================
    def set_link_inertia(self, mass, cg_position, Icg):
        self.m = mass
        self.c = cg_position
        self.I = Icg
    
    def euler_newton(self, q, dq, ddq):
        """ Euler-Netwon recursive algorithm from book Sicilliano """
        ndof = self.ndof
        lq = [len(vec) for vec in [q, dq, ddq]]
        if not np.all(np.array(lq) == ndof):
            raise ValueError("Input vectors must have same length, not: " + str(lq))
        
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
            W[k], dW[k], A[k], Ac[k] = self._fw_prop(i, q[i], dq[i], ddq[i],
                                                     W[k-1], dW[k-1], A[k-1])
        
        # initialize some more vectors (OTHER INDEX DEFINITION than W, dW, ...)
        # i = ndof+1 -> end effector
        # i = 0 - > first link
        # assume no end effector force or torque
        F   = np.zeros((ndof + 1, 2))
        M   = np.zeros(ndof + 1)
        tau = np.zeros(ndof) # joint force or torque
        
        # backward recursion for forces and torques
        for k in np.flip(np.arange(ndof), 0):
            i = k # link index in robot parameters
            F[k], M[k], tau[k] = self._bw_prop(i, q[i],
                                             F[k+1], M[k+1],
                                             dW[k+1], Ac[k+1])
        return tau
    
    def _fw_prop(self, ib, qb, dqb, ddqb, wa, dwa, aa):
        """ Forward propagation of speed and velocity from link i to i+1 """
        
        cb = self.c[ib]
        if self.jt[ib] == 'r':
            db    = self.d[ib]
            arel  = [-(wa + dqb)**2 * db, (dwa + ddqb)*db]
            acrel = [-(wa + dqb)**2 * cb, (dwa + ddqb)*cb]
            wb    = wa  + dqb
            dwb   = dwa + ddqb
            R = self._R_link(ib, qb).T
        elif self.jt[ib] == 'p':
            db    = qb
            arel  = [-wa**2 * db + ddqb, dwa*db + 2*wa*dqb]
            acrel = [-wa**2 * cb + ddqb, dwa*cb + 2*wa*dqb]
            wb    = wa
            dwb   = dwa
            R = self._R_link(ib, self.a[ib]).T
        else:
            raise ValueError("wrong joint type: " + self.jt[ib])
        
        ab = np.dot(R, aa) + np.array(arel)
        ac = np.dot(R, aa) + np.array(acrel)
        return wb, dwb, ab, ac
    
    def _bw_prop(self, ia, qb, Fb, Mb, dwa, aca):
        """ Backward force and torque propagation from i+1 to i """
        
        if self.jt[ia] == 'r':
            # transforme Fb to frame of link a (given in frame b)
            R = self._R_link(ia+1, qb)
            Fbt = np.dot(R, Fb)
            # calculate Fa and Ma
            Fa = Fbt + self.m[ia] * aca
            Ma = Mb  + self.c[ia] * Fa[1]
            Ma += (self.d[ia] - self.c[ia]) * Fb[1]
            Ma += self.I[ia] * dwa
            # joint torque
            taua = Ma
        elif self.jt[ia] == 'p':
            # transforme Fb to frame of link a (given in frame b)
            R = self._R_link(ia+1, self.a[ia+1])
            Fbt = np.dot(R, Fb)
            # calculate Fa and Ma
            Fa = Fbt + self.m[ia] * aca
            Ma = Mb  + self.c[ia] * Fa[1]
            Ma += (qb - self.c[ia]) * Fb[1]
            Ma += self.I[ia] * dwa
            # joint force
            taua = Fa[0]
        else:
            raise ValueError("wrong joint type: " + self.jt[ia])
        
        return Fa, Ma, taua
    
    def _R_link(self, i, qi):
        """ rotation matrix of link i relative to link i-1 """
        if (i+1) >= self.ndof:
            # frame i is the end effector frame
            # end effector pose not implemented
            return np.eye(2)
        else:
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

class Robot_3R(Robot):
    """ Wrapper class for base robot class for 3R robot.
    
    This class adds analytical inverse kinematics for this specific robot.
    """
    def __init__(self, link_length):
        """ Simplified constructor for this 3R robot
        
        Parameters
        ----------
        link_length : list or np.array of floats
            The lengths for the three links.
        """
        if not (len(link_length) == 3):
            raise ValueError("This robot has 3 links, not: " + str(len(link_length)))
        super().__init__(['r', 'r', 'r'], link_length, [0, 0, 0])
    
    def ik(self, p, tol=1e-12):
        """ Analytic inverse kinematics for 3 link robot
        
        Parameters
        ----------
        p : list or np.ndarray of floats
            End-effector pose (x, y, angle)
        tol : float
            Tolerance when a result is assumed zero. (default = 1e-12)
        
        Returns
        -------
        dict
            A dictionary with a key 'success' reporting True if the pose is
            reachable and a key 'q' reporting the different joint solutions
            as a list of numpy arrays.
            If 'success' is False, a key 'info' containts extra info.
        
        Notes
        -----
        Joint limits have to be added.
        """
        # tol = tolerance needed for boundary cases
        # base transform only translation for now TODO
        p = np.array(p)
        p[:2] = p[:2] - self.base[:2]
        
        # define variables for readability
        l1, l2, l3 = (self.d[0], self.d[1], self.d[2])
        x, y, phi = (p[0], p[1], p[2])
        
        # initialize output
        q_up = np.zeros(3)
        q_do = np.zeros(3)
        reachable = False
        
        # start calculations
        if (l1 + l2 + l3) >= np.sqrt(x**2 + y**2):
            # coordinates of end point second link (w)
            pwx = x - l3 * np.cos(phi)
            pwy = y - l3 * np.sin(phi)
            rws = pwx**2 + pwy**2
            if (l1 + l2) >= np.sqrt(rws):
                reachable = True
                # calculate q2
                c2 = (rws - l1**2 - l2**2) / (2*l1*l2)
                # if c2 exactly 1, it can be a little bit bigger at this point
                # because of numerical error, then rescale
                # TODO far from all edge cases are catched
                if abs(c2 - 1) < tol:
                    c2 = np.sign(c2) * 1.0
                    s2 = 0.0
                    q_up[1] = np.arctan2(0, c2) # elbow up
                    q_do[1] = -np.arctan2(0, c2) # elbow down
                else:
                    s2 = np.sqrt(1 - c2**2)
                    q_up[1] = np.arctan2(s2, c2) # elbow up
                    q_do[1] = np.arctan2(-s2, c2) # elbow down
                # calculate q1
                temp = (l1 + l2 * c2)
                s1_up = (temp * pwy - l2 * s2 * pwx) / rws
                c1_up = (temp * pwx + l2 * s2 * pwy) / rws
                s1_do = (temp * pwy + l2 * s2 * pwx) / rws
                c1_do = (temp * pwx - l2 * s2 * pwy) / rws
                q_up[0] = np.arctan2(s1_up, c1_up)
                q_do[0] = np.arctan2(s1_do, c1_do)
                # finally q3
                q_up[2] = phi - q_up[0] - q_up[1]
                q_do[2] = phi - q_do[0] - q_do[1]

        if reachable:
            # check if we have to identical solutions
            if np.allclose(q_up, q_do):
                return {'success': True, 'q': [q_up]}
            else:
                return {'success': True, 'q': [q_up, q_do]}
        else:
            return {'success': False, 'info': "unreachable"}

class Robot_2P(Robot):
    """ Wrapper class for base robot class for 2P robot.
    
    A simple cartesian robot with inverse kinematics.
    First joint along x-axis, second joint along y-axis.
    """
    def __init__(self, link_length):
        """ Simplified constructor for this 2P robot
        
        Parameters
        ----------
        link_length : list or np.array of floats
            The lengths for the two links.
        """
        if not (len(link_length) == 2):
            raise ValueError("This robot has 2 links, not: " + str(len(link_length)))
        super().__init__(['p', 'p'], link_length, [0, np.pi / 2])
    
    def ik(self, p, tol=1e-12):
        """ Inverse kinematics for 2R robot, ignoring orientation.
        
        Joint limits not implemented yet.
        
        Parameters
        ----------
        p : list or np.ndarray of floats
            End-effector pose (x, y, angle)
            Angle (orientation) not used but included for consistency.
        tol : float
            Tolerance when a result is assumed zero. (default = 1e-12)
            Not used but included for consistency.
        
        Returns
        -------
        dict
            A dictionary with a key 'success' reporting True if the pose is
            reachable and a key 'q' reporting the different joint solutions
            as a list of numpy arrays.
            If 'success' is False, a key 'info' containts extra info.
        """
        return {'success': True, 'q': [np.array([p[0], p[1]])]}
        



