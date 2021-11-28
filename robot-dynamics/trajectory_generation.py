import modern_robotics as mr
import numpy as np

# Problem 2
# (10/T^3)*t^3 - (15/T^4)*t^4 + (6/T^5)*t^5

# Problem 5
output5 = mr.QuinticTimeScaling(5,3)
print("\nProblem 5:", output5)

# Problem 6
Xstart = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]])
Xend = np.array([[0,0,1,1], [1,0,0,2], [0,1,0,3], [0,0,0,1]])
Tf = 10
N = 10
method = 3
output6 = mr.ScrewTrajectory(Xstart, Xend, Tf, N, method)
print("\nProblem 6:", output6)

# Problem 7
method = 5
output7 = mr.CartesianTrajectory(Xstart, Xend, Tf, N, method)
print("\nProblem 7:", output7)