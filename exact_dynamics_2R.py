#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 14:02:26 2017

@author: jeroen
"""
from numpy import array, sin, cos, dot

class Exact_dynamics_2R:
    def __init__(self, robot):
        #Define kinematic parameters
        self.l1  = robot.d[0]; #length
        self.l2  = robot.d[1];
        
        self.m1  = robot.m[0]; #mass
        self.m2  = robot.m[1];
        
        self.I1  = robot.I[0]; #inertia around COG
        self.I2  = robot.I[1];
        
        self.lc1 = robot.c[0];#COG
        self.lc2 = robot.c[1];

        #grav = 9.81; #gravitational constant
        
        #Define dynamic parameters
        m1, m2, lc1, lc2, I1, I2 = self.m1, self.m2, self.lc1, self.lc2, self.I1, self.I2
        self.mu11 = m1;
        self.mu21 = m1*lc1;
        self.mu31 = m1*lc1**2+I1;
        
        self.mu12 = m2;
        self.mu22 = m2*lc2;
        self.mu32 = m2*lc2**2+I2;
    
    def run(self, q, dq, ddq):
        return dot(self.M(q), ddq) + dot(self.C(q, dq), dq)

    #Define matrices describing robot dynamics as matlab functions
    def M(self, q):
        l1, mu12, mu22, mu32, mu31 = self.l1, self.mu12, self.mu22, self.mu32, self.mu31
        return array([[mu12*l1**2 + mu22*2*l1*cos(q[1]) + mu31 + mu32,
                          mu32 + mu22*l1*cos(q[1])],
                         [mu32 + mu22*l1*cos(q[1]),
                          mu32]])
          
    def C(self, q, dq):
        l1, mu22 = self.l1, self.mu22
        return array([[-mu22*l1*sin(q[1])*dq[1],
                       -mu22*l1*sin(q[1])*(dq[0]+dq[1])],
                       [mu22*l1*sin(q[1])*dq[0],
                        0]])
        
#def G(q):
#    return grav*[mu12*l1*cos(q(1))+mu21*cos(q(1))+mu22*cos(q(1)+q(2)); mu22*cos(q(1)+q(2))];

if __name__ == "__main__":
    print("testing")
    q = array([1, 2])
    dq = array([1, 2])
    ddq = array([1, 2])