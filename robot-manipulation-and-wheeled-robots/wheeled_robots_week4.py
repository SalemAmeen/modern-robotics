import numpy as np
import modern_robotics as mr

# Problem 3
v = 10*np.cos(np.pi/4)
omega = 10*np.sin(np.pi/4)/2
print("\nProblem 3:", [round(v,2), round(omega,2)])

# Set up H matrix
def H_matrix(y, r_i, B_i, x_i, y_i, phi):
    m1 = np.array([[1/r_i, np.tan(y)/r_i]])
    m2 = np.array([[np.cos(B_i), np.sin(B_i)],[-np.sin(B_i), np.cos(B_i)]])
    m3 = np.array([[-y_i,1,0],[x_i,0,1]])
    m4 = np.array([[1,0,0],[0,np.cos(phi),np.sin(phi)],[0,-np.sin(phi), np.cos(phi)]])

    H = m1 @ m2 @ m3 @ m4
    
    return H

H = np.ones((4,3))
l,w = 3,2

# H_i for each wheel
h1 = H_matrix(-np.pi/4,1,0,l,w,0)
H[0,:] = h1[0]
h2 = H_matrix(np.pi/4,1,0,l,-w,0)
H[1,:] = h2[0]
h3 = H_matrix(-np.pi/4,1,0,-l,-w,0)
H[2,:] = h3[0]
h4 = H_matrix(np.pi/4,1,0,-l,w,0)
H[3,:] = h4[0]

#print("\nH matrix:\n", H)

# Problem 4
u = np.array([-1.18,0.68,0.02,-0.52]).T
H_inv = np.linalg.pinv(H)
Vb = H_inv @ u
print("\nProblem 4:", np.array2string(Vb, separator=','))

# Problem 5
q_k = np.array([0,0,0]).T
phi = q_k[0]
omega_bz = Vb[0]
v_bx = Vb[1]
v_by = Vb[2]
# Set up delta_qb matrix
delta_xb = (v_bx*np.sin(omega_bz) + v_by*(np.cos(omega_bz) - 1)) / omega_bz
delta_yb = (v_by*np.sin(omega_bz) + v_bx*(1 - np.cos(omega_bz))) / omega_bz
delta_qb = np.array([omega_bz, delta_xb, delta_yb]).T
delta_q = np.array([[1,0,0],[0,np.cos(phi),-np.sin(phi)],[0,np.sin(phi),np.cos(phi)]]) @ delta_qb
# Current state
q_k1 = q_k + delta_q
print("\nProblem 5:", np.array2string(np.around(q_k1,3), separator=','))

# Problem 6
T_be = np.array([[0,-1,0,2],[1,0,0,3],[0,0,1,0],[0,0,0,1]])
T_eb = np.linalg.inv(T_be)
Ad_Teb = mr.Adjoint(T_eb)
F6 = np.array([[0,0],[0,0],[-0.25,0.25],[0.25,0.25],[0,0],[0,0]])
J_base = Ad_Teb @ F6
J_rightwheel = J_base[2:5,1]
print("\nProblem 6:", np.array2string(np.around(J_rightwheel,2), separator=','))

# Problem 7
Blist = np.array([0,0,1,-3,0,0])
thetalist = np.array([np.pi/2])
J_arm = mr.JacobianBody(Blist,thetalist)
print("\nProblem 7:", np.array2string(np.around(J_arm,2), separator=','))