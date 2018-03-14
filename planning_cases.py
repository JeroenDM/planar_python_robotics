import numpy as np

from ppr.robot import Robot, Robot_3R
from ppr.path import TolerancedNumber, TrajectoryPt
from ppr.geometry import Rectangle

#=================================================
# Custom robot definition
# To allow setting joint limits and sampling info
# for the two prismatic joints.
#=================================================
class Robot_2P3R(Robot):
    """ A 3R robot mounted on a 2P cartesian robot
    
    This is the first robot with kinematic redundancy!
    
    Attributes
    ----------
    ik_sample : int
            How many samples should be taken in the range of the first two
            joints. Resulting in n_sample * n_sample times the normal number
            of solutions for a 3R robot.
    """
    def __init__(self, link_length, joint_limits=None, ik_samples=None):
        """ Simplified constructor for this 2P3R robot
        
        Add a helper robot to self.sub_robot to reuse the inverse kinematics
        implementation of the 3R robot.
        
        Parameters
        ----------
        link_length : list or np.array of floats
            The lengths for the two links.
        """
        if not (len(link_length) == 5):
            raise ValueError("This robot has 5 links, not: " + str(len(link_length)))
        super().__init__(['p', 'p', 'r', 'r', 'r'],
                         link_length,
                         [np.pi / 2, -np.pi / 2, 0, 0, 0])
        # create 3R robot for inverse kinematics
        self.sub_robot = Robot_3R(link_length[2:])
        
        if joint_limits != None:
            self.jl = joint_limits
        else:
            self.jl = [[0, 2], [0, 2]]
        
        if ik_samples != None:
            self.ik_samples = ik_samples
        else:
            self.ik_samples = [5, 5]

    def ik(self, p):
        """ Discretised / sampled inverse kinematics
        
        This robots has redundance (ndof = 5) compared to the task (3) and
        therefore two joints are sampled in a range to return a sampled
        subset of the infinite solutions that exists for the given pose.
        
        Parameters
        ----------
        p : list or np.ndarray of floats
            End-effector pose (x, y, angle)
        
        Returns
        -------
        dict
            A dictionary with a key 'success' reporting True if the pose is
            reachable and a key 'q' reporting the different joint solutions
            as a list of numpy arrays.
            If 'success' is False, a key 'info' containts extra info.
        """
        jl1, jl2 = self.jl[0], self.jl[1]
        n1 = (jl1[0] + jl1[1]) / 2
        n2 = (jl2[0] + jl2[1]) / 2
        q1 = TolerancedNumber(n1, jl1[0], jl1[1], samples=self.ik_samples[0])
        q2 = TolerancedNumber(n2, jl2[0], jl2[1], samples=self.ik_samples[1])
        grid = np.meshgrid(q1.range, q2.range)
        grid = [ grid[i].flatten() for i in range(2) ]
        grid = np.array(grid).T
        
        q_sol = []
        for qf in grid:
            s = self.ik_fixed_joints(p, q_fixed=qf)
            if s['success']:
                for qi in s['q']:
                    q_sol.append(qi)
        if len(q_sol) > 0:
            return {'success': True, 'q': q_sol}
        else:
            return {'success' : False, 'info': "unreachable"}
    
    def ik_fixed_joints(self, p, q_fixed = [0, 0]):
        """ wrapper function to solve the ik for the last three joints.
        
        The joint solution of the 3R robot is extended with the joint values
        for the fixed joints.
        
        Parameters
        ----------
        p : list or np.ndarray of floats
            End-effector pose (x, y, angle) 
        dict
            A dictionary with a key 'success' reporting True if the pose is
            reachable and a key 'q' reporting the different joint solutions
            as a list of numpy arrays.
            If 'success' is False, a key 'info' containts extra info.
        """
        q_base = [0, 0, 0]
        q_base[0] = q_fixed[1]
        q_base[1] = q_fixed[0]
        self.sub_robot.base = q_base
        sub_sol = self.sub_robot.ik(p)
        if sub_sol['success']:
            # add fixed joints to solution
            q_sol = []
            for qi in sub_sol['q']:
                q_sol.append([*q_fixed, *qi])
            sub_sol['q'] = q_sol
        return sub_sol



#=================================================
# Case 1
#=================================================
robot1 = Robot_2P3R([1, 1, 0.5, 0.5, 0.3])

N1 = 6
x1 = TolerancedNumber(1, 0.9, 1.1, samples=3)
y1 = np.linspace(0.3, 0.6, N1)
a1 = TolerancedNumber(0.0, -0.5, 0.5, samples=5)

path1 = [TrajectoryPt([x1, y1[i], a1]) for i in range(N1)]

scene1 = [Rectangle(0.0, 0.4, 0.1, 0.2, -0.3),
          Rectangle(0.2, 0.8, 0.1, 0.5, 0.2)]

#=================================================
# Case 2
#=================================================
robot2 = Robot_2P3R([1, 1, 2, 1.2, 1], joint_limits=[[0.5, 1], [2, 3]], ik_samples=[3, 5])

N2 = 5
x2 = TolerancedNumber(0.5, 0.3, 0.8, samples=4)
y2 = np.linspace(2, 2.5, N2)
a2 = TolerancedNumber(0.0, -np.pi, np.pi, samples=60)

path2 = [TrajectoryPt([x2, y2[i], a2]) for i in range(N2)]

scene2 = [Rectangle(1, 1, 1, 1.5, 0),
          Rectangle(3, 1, 1, 2.2, 0),
          Rectangle(0, 3.2, 4, 0.5, 0),
          Rectangle(0, 1, 0.2, 3.2, 0),
          Rectangle(0.2, 1, 0.8, 0.5, 0)]