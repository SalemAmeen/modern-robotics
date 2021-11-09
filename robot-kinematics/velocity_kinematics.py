import modern_robotics as mr
import numpy as np
from numpy import linalg as LA

# Problem 1
F_s = np.array([0,0,0,2,0,0])
S_s = np.array([[0,0,1,0,0,0],
                [0,0,1,0,-1,0],
                [0,0,1,0,-2,0]])
theta = np.array([0,np.pi/4, 0])
J_s = mr.JacobianSpace(np.transpose(S_s),theta)
torque = np.dot(np.transpose(J_s), F_s)
print("\nProblem 1: \n", np.array2string(torque, separator=','))

# Problem 2
F_b = np.array([0,0,10,10,10,0])
theta = np.array([0,0,np.pi/2, -np.pi/2])
S_b = np.array([[0,0,1,0,4,0],
                [0,0,1,0,3,0],
                [0,0,1,0,2,0],
                [0,0,1,0,1,0]])
J_b = mr.JacobianBody(np.transpose(S_b), theta)
torque = np.dot(np.transpose(J_b), F_b)
print("\nProblem 2: \n", np.array2string(torque, separator=','))

# Problem 3
theta = np.array([np.pi/2, np.pi/2, 1])
S = np.array([[0,0,1,0,0,0],
            [1,0,0,0,2,0],
            [0,0,0,0,1,0]])
J_s = mr.JacobianSpace(np.transpose(S), theta)
print("\nProblem 3: \n", np.array2string(J_s, separator=','))

# Problem 4
theta = np.array([np.pi/2, np.pi/2, 1])
B = np.array([[0,1,0,3,0,0],
            [-1,0,0,0,3,0],
            [0,0,0,0,0,1]])
J_b = mr.JacobianBody(np.transpose(B), theta)
print("\nProblem 4: \n", np.array2string(J_b, separator=','))

# Problem 5, from section 5.4 in textbook
theta = (np.pi/2) * np.ones((1,7))
J_b = np.array([[0,-1,0,0,-1,0,0],
                [0,0,1,0,0,1,0],
                [1,0,0,1,0,0,1],
                [-0.105,0,0.006,-0.045,0,0.006,0],
                [-0.889,0.006,0,-0.844,0.006,0,0],
                [0,-0.105,0.889,0,0,0,0]])
J_v = J_b[3:,:]
A = np.dot(J_v, np.transpose(J_v))
w,v = LA.eig(A) # w: eigenvalues, v: eigenvectors
# Directions of the principal axes of ellipsoid are the eigenvectors
# Lengths of principal semi-axis are sqrt(eigenvalues)
idx = np.where(w == np.amax(w)) # index of longest principal semi-axis
axis_dir = v[idx[0][0]]
print("\nProblem 5: \n", np.array2string(axis_dir, separator=','))

# Problem 6
axis_length = np.sqrt(w[idx[0][0]])
print("\nProblem 6: \n", np.array2string(axis_length, separator=','))