import modern_robotics as mr
import numpy as np

# Problem 1
M = np.array([[1,0,0,3.732],[0,1,0,0],[0,0,1,2.732],[0,0,0,1]])
print("\n", "Problem 1:", "\n", np.array2string(M, separator=','))

# Problem 2
S = np.transpose(np.array([[0,0,1,0,-1,0],
                          [0,1,0,0,0,1],
                          [0,1,0,1,0,2.732],
                          [0,1,0,-0.73,0,3.732],
                          [0,0,0,0,0,1],
                          [0,0,1,0,-3.732,0]]))
print("\n", "Problem 2:", "\n", np.array2string(S, separator=','))

# Problem 3
B = np.transpose(np.array([[0,0,1,0,2.73,0],
                          [0,1,0,2.73,0,-2.73],
                          [0,1,0,3.73,0,-1],
                          [0,1,0,2,0,0],
                          [0,0,0,0,0,1],
                          [0,0,1,0,0,0]]))
print("\n", "Problem 3:", "\n", np.array2string(B, separator=','))

# Problem 4
theta = np.array([-np.pi/2, np.pi/2, np.pi/3, -np.pi/4, 1, np.pi/6])
FK_s = np.around(mr.FKinSpace(M, S, theta), 4)
print("\n", "Problem 4:", "\n", np.array2string(FK_s, separator=','))

# Problem 5
FK_b = np.around(mr.FKinBody(M, B, theta), 4)
print("\n", "Problem 5:", "\n", np.array2string(FK_b, separator=','))