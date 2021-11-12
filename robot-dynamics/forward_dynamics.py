import modern_robotics as mr
import numpy as np

# UR5 Parameters
M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])

G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])

Glist = np.array([G1, G2, G3, G4, G5, G6])
Mlist = np.array([M01, M12, M23, M34, M45, M56, M67])
Slist = np.array([[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]])

# Given in problem:
thetalist = np.array([0, np.pi/6, np.pi/4, np.pi/3, np.pi/2, 2*np.pi/3])
thetadot = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
theta_ddot = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
g = np.array([0,0,-9.81])
F_tip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

# Problem 1
output1 = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
print("\nProblem 1:\n", np.array2string(np.around(output1, 3), separator=','))

# Problem 2
output2 = mr.VelQuadraticForces(thetalist, thetadot, Mlist, Glist, Slist)
print("\nProblem 2:\n", np.array2string(np.around(output2, 3), separator=','))

# Problem 3
output3 = mr.GravityForces(thetalist, g, Mlist, Glist, Slist)
print("\nProblem 3:\n", np.array2string(np.around(output3, 3), separator=','))

# Problem 4
output4 = mr.EndEffectorForces(thetalist, F_tip, Mlist, Glist, Slist)
print("\nProblem 4:\n", np.array2string(np.around(output4, 3), separator=','))

# Problem 5
taulist = np.array([0.0128, -41.1477, -3.7809, 0.0323, 0.0370, 0.1034])
output5 = mr.ForwardDynamics(thetalist, thetadot, taulist, g, F_tip, Mlist, Glist, Slist)
print("\nProblem 5:\n", np.array2string(np.around(output5, 3), separator=','))