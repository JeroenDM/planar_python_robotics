#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 29 14:07:09 2018

@author: jeroen
"""

class UnknownNumber:
    def __init__(self):
        pass

class Pos:
    def __init__(self, pxy):
        self.iun = [isinstance(pxy[i], UnknownNumber) for i in range(2)]
        self.p = pxy

path = [Pos([0, 0]), Pos([UnknownNumber(), 0.5]), Pos([2, 1])]
road = [[-1, -2], [0, 3], [5, 9]]

class Car:
    def __init__(self, path):
        self.path = path
        
    def fk(self, rxy):
        return [rxy[0], rxy[1] + 1]
        
    def create_error_function(self):
        def er(road):
            return "Unkown road value"
        
        return er
    
    def create_deviate_function(self, ip, pp, jp):
        
        def dev(road):
            val = self.fk(road[ip])
            return val[jp] - pp.p[jp]

        return dev
    
    def set_dev_function(self):
        c = []
        for i, pp in enumerate(self.path):
            for j in range(2):
                if pp.iun[j]:
                    f = self.create_error_function()
                    c.append({'fun': f})
                else:
                    f = self.create_deviate_function(i, pp, j)
                    c.append({'fun': f})
        self.c = c

car = Car(path)
#df = car.create_deviate_function()

#print(df(road))

print([-1, -2, -1, 2.5, 3, 8])

car.set_dev_function()
for i in range(len(car.c)):
    print(car.c[i]['fun'](road))