import time
import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import RobotManyDofs
from ppr.path import TrajectoryPt, TolerancedNumber
from ppr.geometry import Rectangle
from ppr.sampling import cart_to_joint, cart_to_joint_dynamic
from ppr.sampling import get_shortest_path

# ROBOT
robot1 = RobotManyDofs(8, link_length=1)
robot1.set_joint_limits([(-np.pi / 2, np.pi / 2)] * 8)

robot1.lw = [0.15] * robot1.ndof
robot1.collision_shapes = robot1.get_shapes([0] * robot1.ndof)

# PATH
# dx    = TolerancedNumber(2, 0.3, 0.8, samples=3)
dx = np.ones(5) * 5
dy = np.linspace(1.3, 2.5, 5)
angle = TolerancedNumber(0.0, -np.pi / 2, np.pi / 2, samples=10)
path1 = [TrajectoryPt([dx[i], dy[i], angle]) for i in range(5)]

# COLLISION SCENE
sc1 = []
sc1.append(Rectangle(0, -1, 5, 0.2, 0))  # horizontal lower
sc1.append(Rectangle(0, 3, 5, 0.2, 0))  # horizontal upper
sc1.append(Rectangle(1, -1, 0.2, 2.5, 0))  # vertical left
sc1.append(Rectangle(2.5, 1, 0.2, 2, 0))  # vertical middle
sc1.append(Rectangle(4, -1, 0.2, 2.5, 0))  # vertical right

q_random = [0.1] * 8

fig1, ax1 = plt.subplots()
plt.title("The Robot")
ax1.axis("equal")
robot1.plot(ax1, q_random, "k")
for tp in path1:
    tp.plot(ax1)
for r in sc1:
    r.plot(ax1, "g")
plt.show()

robot1.ik_samples = [7] * 5

start_time = time.time()

# path_js = cart_to_joint(robot1, path1, check_collision=True, scene=sc1)
# path_js = cart_to_joint(robot1, path1, check_collision=True, scene=sc1, method='random',N_cart=10,N_red_joints=125)
# path_js = cart_to_joint(robot1, path1, check_collision=True, scene=sc1, method='halton',N_cart=10,N_red_joints=125)

# path_js = cart_to_joint(robot1, path1, check_collision=True, scene=sc1)
settings = {
    "max_iters": 20,
    "min_js": 100,
    "js_inc": 10,
    "red_js_inc": 100,
    "ik_sampling_method": "halton",
}
path_js = cart_to_joint_dynamic(
    robot1, path1, check_collision=True, scene=sc1, parameters=settings
)

print([len(qp) for qp in path_js])
sol = get_shortest_path(path_js)
print(sol["success"])

print("Total time: " + str(time.time() - start_time))
print("Path length: " + str(sol["length"]))

fig2, ax2 = plt.subplots()
ax2.axis("equal")
# robot1.plot_path_kinematics(ax2, path_js[2])
robot1.plot_path(ax2, sol["path"])
for r in sc1:
    r.plot(ax2, "k")
for tp in path1:
    tp.plot(ax2)

plt.title("8 dof revolute arm")
plt.show()
