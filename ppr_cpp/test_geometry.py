import geometry
import numpy as np

r = geometry.Rectangle(1, 2, 0, 0, 0.2)
r2 =geometry.Rectangle(1, 2, 0, 0, 0)
print(r.in_collision(r2))

pts = r2.get_plot_points()
print(pts)

class Rectangle(geometry.Rectangle):
    """Wrap the cpp class for visualisation"""

    def plot(self, axes_handle, *arg, **karg):
        """Plot the rectangle on axes"""
        p = self.get_plot_points()
        p = np.vstack((p, p[0]))
        axes_handle.plot(p[:, 0], p[:, 1], *arg, **karg)

print("-----test random rectangles-----")

# test random set of rectangles
nr = 20
r_pos = np.random.rand(nr, 2) * 10 - 5
r_sha = np.random.rand(nr, 2) * 6 - 3
r_ang = np.random.rand(nr) * np.pi - np.pi/2
rectangles = []
for i in range(nr):
    rectangles.append(Rectangle(r_sha[i, 0], r_sha[i, 1],
                                r_pos[i, 0], r_pos[i, 1],
                                r_ang[i]))
import matplotlib.pyplot as plt
fig2 = plt.figure(2)
ax2 = fig2.gca()
plt.axis('equal')
plt.axis([-10, 10, -10, 10])
for rect_a in rectangles:
    col_a = False
    for rect_b in rectangles:
        if rect_a != rect_b:
            if rect_a.in_collision(rect_b):
                col_a = True
                break
    if col_a:
        print("collision!")
        rect_a.plot(ax2, 'r')
    else:
        print("oof")
        rect_a.plot(ax2, 'g')
# plt.show()