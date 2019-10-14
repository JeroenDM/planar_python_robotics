import time
import numpy as np
import matplotlib.pyplot as plt
from ppr.robot import RobotManyDofs
from ppr.path import TrajectoryPt, TolerancedNumber
from ppr.geometry import Rectangle
from ppr.sampling import cart_to_joint, cart_to_joint_dynamic
from ppr.sampling import get_shortest_path, iterative_bfs

# ROBOT
robot1 = RobotManyDofs(6, link_length=2)
robot1.set_joint_limits([(-np.pi / 2, np.pi / 2)] * 6)
robot1.do_check_self_collision = True

# PATH line in gap
N = 5
G = 0.5
x = G / 2 * np.ones(N)
y = np.linspace(4, 6, N)
a = TolerancedNumber(0, 0, np.pi, samples=20)
path1 = [TrajectoryPt([x[i], y[i], a]) for i in range(N)]

# collision scene small gap width G
sc1 = []
sc1.append(Rectangle(-2, 4, 2, 4, 0))
sc1.append(Rectangle(G, 4, 2, 4, 0))

q_random = [0.1] * robot1.ndof

# fig1, ax1 = plt.subplots()
# plt.title("The Robot")
# ax1.axis("equal")
# robot1.plot(ax1, q_random, "k")
# for tp in path1:
#     tp.plot(ax1)
# for r in sc1:
#     r.plot(ax1, "g")
# plt.show()

robot1.ik_samples = [9, 9, 9]  # 3 of the 6 joints redundant


def change_robot_link_width(robot, new_width):
    robot.lw = [new_width] * robot.ndof
    robot.collision_shapes = robot.get_shapes([0] * robot.ndof)


def run_case(robot, path, scene):
    start_time = time.time()
    js = cart_to_joint(
        robot,
        path,
        check_collision=True,
        scene=scene,
        method="halton",
        N_cart=100,
        N_red_joints=100,
    )
    sol = get_shortest_path(js, method="dijkstra")
    run_time = time.time() - start_time

    if sol["success"]:
        return sol, run_time
    else:
        print("Failed at solving with method: " + "halton")
        print([len(qp) for qp in js])
        return None, None


for w in [0.05, 0.1, 0.2]:
    change_robot_link_width(robot1, w)
    sol1, runtime = run_case(robot1, path1, sc1)
    print(f"Runtime for w {w} is {str(runtime)}")

fig2, ax2 = plt.subplots()
ax2.axis("equal")
# robot1.plot_path_kinematics(ax2, path_js[2])
robot1.plot_path(ax2, sol1["path"])
for r in sc1:
    r.plot(ax2, "k")
for tp in path1:
    tp.plot(ax2)

plt.title("8 dof revolute arm")
plt.show()
