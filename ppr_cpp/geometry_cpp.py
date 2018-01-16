if (__name__ == "ppr_cpp.geometry_cpp"):
    import ppr_cpp.geometry as gm
else:
    import geometry as gm

import numpy as np

class Rectangle(gm.Rectangle):
    """Wrap the cpp class for visualisation"""

    def plot(self, axes_handle, *arg, **karg):
        """Plot the rectangle on axes"""
        p = self.get_plot_points()
        p = np.vstack((p, p[0]))
        axes_handle.plot(p[:, 0], p[:, 1], *arg, **karg)