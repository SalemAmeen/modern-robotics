import modern_robotics as mr
import numpy as np

# Problem 1
density = 5600 # kg/m^3
r_cyl = 0.04/2 # radius of cylinder, m
l_cyl = 0.2 # length of cylinder, m
r_sph = 0.2/2 # radius of sphere, m

# Calculate masses
m_cyl = density * l_cyl * np.pi * r_cyl**2
m_sph = density * (4/3) * np.pi * r_sph**3

# Inertia matrix for cylinder
Ixx_cyl = m_cyl * (0.25*r_cyl**2 + (1/12)*l_cyl**2)
Iyy_cyl = Ixx_cyl
Izz_cyl = 0.5 * m_cyl * r_cyl**2
I_cyl = np.diag([Ixx_cyl, Iyy_cyl, Izz_cyl])

# Inertia matrix for one sphere
Ixx_sph = (2/5) * m_sph * r_sph**2 + m_sph * (r_sph + l_cyl/2)**2
Iyy_sph = Ixx_sph
Izz_sph = (2/5) * m_sph * r_sph**2
I_sph = np.diag([Ixx_sph, Iyy_sph, Izz_sph])

# Total interia matrix for dumbbell
I_total = I_cyl + 2*I_sph
print("\nProblem 1:\n", np.array2string(I_total, separator=','))

# Problem 5

# UR5 parameters:
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

# Inverse Dynamics
joint_torques = mr.InverseDynamics(thetalist, thetadot, theta_ddot, g, F_tip, Mlist, Glist, Slist)
print("\nProblem 5:\n", np.array2string(np.around(joint_torques, 3), separator=','))