#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jan 26 15:51:03 2018

@author: jeroen
"""

# -----------------------------------------------------------------------------
# interference.py
# ----------------------------------------------------------------------------- 
"""
Author:     Jesse M. Kinder
Created:    2016 Apr 15
Modified:   2016 Apr 15

Description
-----------
Build a GUI wrapper to explore the interference pattern of two waves.
"""
import tkinter

class SimplePlot(Tkinter.Canvas):

    def plot(self, x, y):
        self.create_line((x, y, x+1, y), fill="black")

#
# test program

import random
import time


root = Tkinter.Tk()
root.title("demoSimplePlot")

widget = SimplePlot(root)
widget.pack(fill="both", expand=1)

widget.update() # display the widget

data = []
for i in range(5000):
    data.append((random.randint(0, 200), random.randint(0, 200)))

t0 = time.time()

for x, y in data:
    widget.plot(x, y)

widget.update() # make sure everything is drawn

print(time.time() - t0, "seconds")

root.mainloop()