import modern_robotics as mr
import numpy as np

# Problems for Quiz 3.3 and 3.4: Rigid Body Motions

# Set up transformation matrices
T_sa = np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,1],[0,0,0,1]])
T_sb = np.array([[1,0,0,0],[0,0,1,2],[0,-1,0,0],[0,0,0,1]])
T_ab = np.array([[0,-1,0,-1],[-1,0,0,0],[0,0,-1,-2],[0,0,0,1]])

# Problem 1
print("Problem 1: T_sa")
print(T_sa)

# Problem 2
inv2 = mr.TransInv(T_sb)
print("Problem 2: Inverse of T_sb")
print(inv2)

# Problem 3
print("Problem 3: T_ab")
print(T_ab)

# Problem 5
p_b = np.transpose(np.array([1,2,3,1]))
p_s = np.dot(T_sb,p_b)
print("Problem 5: P_s")
print(p_s)

# Problem 7
V_s = np.transpose(np.array([3,2,1,-1,-2,-3]))
Ad_T = mr.Adjoint(mr.TransInv(T_sa))
V_a = np.dot(Ad_T, V_s)
print("Problem 7: V_a")
print(V_a)

# Problem 8
m_sa = mr.MatrixLog6(T_sa)
print("Problem 8: Theta from matrix log of T_sa")
print(mr.AxisAng6(mr.se3ToVec(m_sa))[1])

# Problem 9
vec9 = np.transpose(np.array([0,1,2,3,0,0]))
se3 = mr.VecTose3(vec9)
T9 = mr.MatrixExp6(se3)
print("Problem 9: Matrix exponential")
print(T9)

# Problem 10
F_b = np.transpose(np.array([1,0,0,2,1,0]))
T_bs = mr.TransInv(T_sb)
Ad_Tbs = mr.Adjoint(T_bs)
F_s = np.dot(np.transpose(Ad_Tbs), F_b)
print("Problem 10: F_s")
print(F_s)

# Problem 11
T = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
print("Problem 11: Inverse of T")
print(mr.TransInv(T))

# Problem 12
V = np.transpose(np.array([1,0,0,0,2,3]))
print("Problem 12: se(3) matrix")
print(mr.VecTose3(V))

# Problem 13
print("Problem 13: Normalized screw axis")
print(mr.ScrewToAxis(np.array([0,0,2]), np.array([1,0,0]), 1))

# Problem 14
m14 = np.array([[0,-1.5708,0,2.3562],
                [1.5708,0,0,-2.3562],
                [0,0,0,1],
                [0,0,0,0]])
print("Problem 14: Transformation matrix")
print(mr.MatrixExp6(m14))

# Problem 15
print("Problem 15: Matrix log")
print(mr.MatrixLog6(T))