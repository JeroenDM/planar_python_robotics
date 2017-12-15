import geometry
import numpy as np

r = geometry.Rectangle(1, 2, 0, 0, 0.2)
print(r.area())
r2 =geometry.Rectangle(1, 2, 0, 0, 0)
print(r.in_colission(r2))

from geometry import Rectangle

print("-----test random rectangles-----")

# test random set of rectangles
nr = 4
r_pos = np.random.rand(nr, 2) * 10 - 5
r_sha = np.random.rand(nr, 2) * 6 - 3
r_ang = np.random.rand(nr) * np.pi - np.pi/2
rectangles = []
for i in range(nr):
    rectangles.append(Rectangle(r_sha[i, 0], r_sha[i, 1],
                                r_pos[i, 0], r_pos[i, 1],
                                r_ang[i]))

res = rectangles[0].in_colission(rectangles[1])
print(res)
for rect_a in rectangles:
    for rect_b in rectangles:
        if rect_a.in_colission(rect_b):
            print("collision!")
        else:
            print("oof")