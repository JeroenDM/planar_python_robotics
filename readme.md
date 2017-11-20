# Planar Python Robotics
The main goal is to test robotic algorithms using only numpy and vanilla python. There are great libraries for robot simulation and related task, but installing them is always a hassle and very dependent on operating system and python version.

By only using Python 3.6 (from the [Anaconda](https://www.anaconda.com/download/) distribution) and [SciPy](https://www.scipy.org/) I hope to have a robust framework that will work on any system for the next three years.
I also started using [NetworkX](https://networkx.github.io/), will see if I get into version trouble in the future...

The main drawback is that I have to write a lot of stuff myself. This is why I start with planar robotics. I'm not sure if it is usefull to do this. But it will be fun and I will learn a buch.

## General interface
I try to get simple high level commands to set up a path following problem in 2D. The idea of toleranced trajectory points and the structure of the plannning graph comes from the [Descartes package](http://wiki.ros.org/descartes). A big thing missing there is easy handling of inverse kinematics. I ty to come up with a good high level interface to handle redundant inverse kinematics.

### The path
A path is given as a list of *trajectory points*. Each trajectory point specifies three values: x-position, y-position and orientation. Each of these three values can have tolerances. This is achieved by passing a *toleranced value* instead of a number when creating a trajectory points.
A *toleranced value* has an lower bound, upper bound and nominal value. The nominal value is supposed to be the desired value. It will be used to minimize the deviation from this value when planning. It does not have to be the middle of the interval.

### The robot
For the robot I some kind of adapted Denavit-Hartenberg convention in 2D. I will have to explain it here.

The inverse kinematics is where it get's interesting. The basic assumtion is to only use analytic inverse kinematics to achieve a reasobanle speed. The idea is that for a more than 3 dof robot, *fixed joints* will have to be specified. The *discrete inverse kinematics* function will automaticly sample these joints in there interval of joint limits and return all possible inverse kinematics solutions. The is a method that could quickly explode of not used carefully, having complexilty samples_per_dof^(ndof - 3) Only to be used for rude sampling of possible path.

### The planner
The planner should give an esimate of the required calculation time, to warn the user if the required planning problem is sampled to fine. A planning graph is construced and the shorted path is found.

### The optimizer
After a course path plan is formulated, the trajectory should be furter optimized. An optimization problem if formulated including a dynamic model of the robot and while respecting torque limits.
