import numpy as np

# Set up H matrix

def H_matrix(y, r_i, B_i, x_i, y_i, phi):
    m1 = np.array([[1/r_i, np.tan(y)/r_i]])
    m2 = np.array([[np.cos(B_i), np.sin(B_i)],[-np.sin(B_i), np.cos(B_i)]])
    m3 = np.array([[-y_i,1,0],[x_i,0,1]])
    m4 = np.array([[1,0,0],[0,np.cos(phi),np.sin(phi)],[0,-np.sin(phi), np.cos(phi)]])

    H = np.matmul(m1,m2)
    H = np.matmul(H,m3)
    H = np.matmul(H,m4)
    
    return H

H = np.ones((4,3))

h1 = H_matrix(0,0.25,-np.pi/4,2,2,0)
H[0,:] = h1[0]
h2 = H_matrix(0,0.25,np.pi/4,-2,2,0)
H[1,:] = h2[0]
h3 = H_matrix(0,0.25,3*np.pi/4,-2,-2,0)
H[2,:] = h3[0]
h4 = H_matrix(0,0.25,-3*np.pi/4,2,-2,0)
H[3,:] = h4[0]

print("\nH matrix:\n", H)

# Problem 1
Vb = np.array([1,0,0])
u = np.matmul(H, Vb)
print("\nProblem 1:", np.array2string(u, separator=','))

# Problem 2
Vb = np.array([1,2,3])
u = np.matmul(H, Vb)
print("\nProblem 2:", np.array2string(u, separator=','))

# Problem 3

# vx - vy = u1/2.8
# vx + vy = u2/2.8
# -vx + vy = u3/2.8
# vx + vy = u4/-2.8

# Therefore, u2 = -u4, and u1 = -u3
# To maximize linear speed, set u1 = u2
# Set u to 10 to maximize sqrt(vx^2 + vy^2)
# From solving system of equations, vy = 0
# Therefore, vx = 10/2.8

v_max = 10/2.8
print("\nProblem 3:", v_max)