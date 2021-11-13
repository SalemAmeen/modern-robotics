import modern_robotics as mr
import numpy as np
import csv

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

dthetalist = np.array([0, 0, 0, 0, 0, 0])
g = np.array([0,0,-9.81])
dt = 0.005
intRes = 10

# Open csv files in write mode
f1 = open(r'C:\Users\Summi\Documents\Python\simulation1.csv', 'w', newline='')
writer1 = csv.writer(f1)
f2 = open(r'C:\Users\Summi\Documents\Python\simulation2.csv', 'w', newline='')
writer2 = csv.writer(f2)

# Simulation 1: all joint angles and torques are zero
thetalist1 = np.array([0, 0, 0, 0, 0, 0])
N1 = int(3/dt)
taumat1 = np.zeros((N1,6))
Ftipmat1 = np.zeros((N1, 6))
thetamat1, dthetamat1 = mr.ForwardDynamicsTrajectory(thetalist1, dthetalist, taumat1, g, Ftipmat1, Mlist, Glist, Slist, dt, intRes)

# Simulation 2: joint 2 is -1 radian, torques are still zero
thetalist2 = np.array([0, -1, 0, 0, 0, 0])
N2 = int(5/dt)
taumat2 = np.zeros((N2,6))
Ftipmat2 = np.zeros((N2, 6))
thetamat2, dthetamat2 = mr.ForwardDynamicsTrajectory(thetalist2, dthetalist, taumat2, g, Ftipmat2, Mlist, Glist, Slist, dt, intRes)

# Add joint angles to csv files
for i in range(N1):
    writer1.writerow(thetamat1[i,:])
    pass

for j in range(N2):
    writer2.writerow(thetamat2[j,:])
    pass

# Close the csv files
f1.close()
f2.close()