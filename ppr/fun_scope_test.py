#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 13:18:51 2018

@author: jeroen
"""



def create_fun(n):
    c = []
    for j in range(3):
        if n == 1:
            def f(a):
                print("(n==1) n: " + str(n) + " j: " + str(j) + " a: " + str(a))
            c.append(f)
        else:
            def f(a):
                print("(n==2) n: " + str(n) + " j: " + str(j) + " a: " + str(a))
            c.append(f)
    return c


c1 = create_fun(1)
c2 = create_fun(2)

for i in range(3):
    print(c1[0](88))

for i in range(3):
    print(c2[i](8))



def create_f2(n, j):
    def f(a):
        print("(n==2) n: " + str(n) + " j: " + str(j) + " a: " + str(a))
    return f

def create_f(n, j):
    def f(a):
        print("(n==1) n: " + str(n) + " j: " + str(j) + " a: " + str(a))
    return f
    
def create_fun(n):
    c = []
    for j in range(3):
        if n == 1:
            f = create_f(n, j)
            c.append(f)
        else:
            f = create_f2(n, j)
            c.append(f)
    return c


c1 = create_fun(1)
c2 = create_fun(2)

for i in range(3):
    print(c1[i](99))

for i in range(3):
    print(c2[i](11))