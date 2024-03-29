#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
from scipy.linalg import norm
from .cpp.geometry import Rectangle as BaseRec


def rotation(angle):
    """ Create 2x2 rotation matrix from angle
    
    Parameters
    ----------
    anlge : float
    
    Returns
    -------
    numpy.ndarray
        Rotation matrix as a numpy array
    
    Examples
    --------
    >>> rotation(0.0)
    array([[ 1., -0.],
           [ 0.,  1.]])
    >>> rotation(np.pi / 2)
    array([[  6.12323400e-17,  -1.00000000e+00],
           [  1.00000000e+00,   6.12323400e-17]])
    """
    cos = np.cos(angle)
    sin = np.sin(angle)
    return np.array([[cos, -sin], [sin, cos]])


class Rectangle(BaseRec):
    """ Rectangle plotting, handling and collision detection
    
    Attributes
    ----------
    p : numpy.ndarray
        The four corner points of the rectangle, counter-clockwise, starting
        with the point at the (x, y) position. The array has shape (4, 2).
    
    Examples
    --------
    >>> rect = Rectangle(0.3, 0.4, 1.5, -1.6, 2.7)
    >>> import matplotlib.pyplot as plt
    >>> fig, ax = plt.subplots()
    >>> rect.plot(ax)
    """

    def __init__(self, x, y, dx, dy, angle):
        """ Create a rectangle object
        
        Parameters
        ----------
        x : float
            x-position of rectangle, one of the four corner points.
        y : float
            y-position of rectangle, one of the four corner points.
        dx : float
            Width before rotation.
        dy : float
            Height before roation.
        angle : float
            Angle between x-axis and bottom side of rectangle.
        """
        super().__init__(x, y, dx, dy, angle)
        self.R = rotation(angle)
        self.p = self.get_vertices()
        self.n = self.get_normals()

    def update_pose(self, x, y, angle):
        """ ugly duplicate of c++ set_pose function
        
        Need this to update attributes in python side of this class.
        TODO solve this duplicating issue
        
        Now the whole initialization I wanted to avoid is copied agian.
        
        Parameters
        ----------
        x : float
            x-position of rectangle, one of the four corner points.
        y : float
            y-position of rectangle, one of the four corner points.
        angle : float
            Angle between x-axis and bottom side of rectangle.
        """
        self.set_pose(x, y, angle)
        self.p = self.get_vertices()
        self.n = self.get_normals()
        self.R = rotation(angle)

    def project(self, axis):
        """ Project all points of rectangle on an axis given as unit vector
        
        Use the dot product between the vector from the origin to a corner
        point and the given axis unit vector.
        
        Parameters
        ----------
        axis : numpy.ndarray of floats
            Unit vector representing the axis on wich to project the points.
        
        Returns
        -------
        numpy.ndarray
            Array with shape (4,) containing the projected points with the
            origin as reference.
        
        Examples
        --------
        
        Project simple square on x-axis.
        
        >>> rec1 = Rectangle(0, 0, 1, 1, 0)
        >>> rec1.project(np.array([1, 0]))
        array([ 0.,  1.,  1.,  0.])
        
        Project the same square on diagonal axis.
        
        >>> rec1.project(np.array([0.70710678, 0.70710678]))
        array([ 0.        ,  0.70710678,  1.41421356,  0.70710678])
        """
        return np.dot(self.p, axis)

    def distance(self, rect2, tol=1e-9):
        """ Check if it collides with another rectangle.
        
        Use the separating axis theorem.
        Project both rectangles along all 8 normals and check overlap.
        
        Parameters
        ----------
        rect2 : ppr.geometry.Rectangle
            The other rectangle to calculate the separating distance with.
        
        Returns
        -------
        float
            Distance between the two rectangles. Negative penetration depth
            if in collision
        
        Examples
        --------
        >>> rec1 = Rectangle(0, 0, 1, 1, 0)
        >>> rec2 = Rectangle(0.5, 0, 1, 1, 0.1)
        >>> rec3 = Rectangle(1.5, 0.5, 1, 2, -0.2)
        >>> rec1.distance(rec2)
        -0.59733551733202006
        >>> rec1.distance(rec3)
        0.5
        """
        n1 = self.get_normals()
        n2 = rect2.get_normals()
        n_all = np.vstack((n1, n2))
        # assume collision until proven otherwise
        col = True
        dist = 0.0
        i = 0
        while col and i < 8:
            pr1 = self.project(n_all[i])
            pr2 = rect2.project(n_all[i])
            d1 = min(pr2) - max(pr1)
            d2 = min(pr1) - max(pr2)
            if d1 > tol or d2 > tol:
                col = False
            dist = max(d1, d2)
            i += 1

        return dist

    def get_matrix_form(self):
        """ Get the matrix representation of the rectanlge
        
        A rectangle is described by four linear inequalities.
        If we write these inequalities in matrix form we get
        A * x <= b
        
        x is the vector of all possible x and y coordinates [x; y]
        
        Returns
        -------
        A : numpy.ndarray of floats
            Array with shape (4, 2), every rows contains the coefficient of
            on of the inequalities.
        b : numpy.ndarray fo floats
            Vector of length 4 containing the right hand side of the
            inequalities.
            
        Examples
        --------
        
        Show the inequalities for a trivial case.
        For a 1x2 rectange at the origin whe whould expect:
        -y <= 0
        x <= 1
        y <= 2
        -x <= 0
        
        >>> rec1 = Rectangle(0, 0, 1, 2, 0)
        >>> A1, b1 = rec1.get_matrix_form()
        >>> A1_desired = np.array([[0, -1], [1, 0], [0, 1], [-1, 0]])
        >>> np.allclose(A1, A1_desired)
        True
        >>> np.round(b1, 15)
        array([ 0.,  1.,  2.,  0.])
        """
        A = self.n
        # row wise dot product
        b = np.sum(A * self.p, axis=1)
        return A, b

    def plot(self, ax, *arg, **karg):
        p = self.get_vertices()
        p = np.vstack((p, p[0]))
        ax.plot(p[:, 0], p[:, 1], *arg, **karg)
