import modern_robotics as mr
import numpy as np
import csv

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev,writer):
    """Computes inverse kinematics in the body frame for an open chain robot

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        print("\nIteration", i+1, ":\n")
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        # Add joint angles to csv file
        writer.writerow(thetalist)
        # Print joint vectors
        print("Joint vector:", np.array2string(np.around(thetalist, 4), separator=','), "\n")
        i = i + 1
        Vb \
        = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                       thetalist)), T)))
        # Print end effector configuration and errors
        print("SE(3) end-effector config:\n", np.around(mr.FKinBody(M, Blist, \
                                                       thetalist), 4), "\n")
        print("Error twist V_b:", np.around(Vb, 4), "\n")
        print("Angular error magnitude:", np.around(np.linalg.norm([Vb[0], Vb[1], Vb[2]]), 4), "\n")
        print("Linear error magnitude:", np.around(np.linalg.norm([Vb[3], Vb[4], Vb[5]]), 4), "\n")

        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err)

# Inverse Kinematics Project

# Open csv file in write mode and create writer
f = open(r'C:\Users\Summi\Documents\Python\iterates.csv', 'w')
writer = csv.writer(f)

# UR5 dimensions (m)
W1 = 0.109
W2 = 0.082
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095

# Screw axes in the end-effector frame
Blist = np.array([[0,1,0,W1+W2,0,L1+L2],
                [0,0,1,H2,-L1-L2,0],
                [0,0,1,H2,-L2,0],
                [0,0,1,H2,0,0],
                [0,-1,0,-W2,0,0],
                [0,0,1,0,0,0]])
Blist = np.transpose(Blist)

# End effector frame in the zero position
M = np.array([[-1,0,0, L1+L2],
            [0,0,1, W1+W2],
            [0,1,0, H1-H2],
            [0,0,0,1]])

# Desired end effector configuration
T = np.array([[0,1,0,-0.5],
            [0,0,-1,0.1],
            [-1,0,0,0.1],
            [0,0,0,1]])

# Tolerances for orientation and linear error
eomg = 0.001
ev = 0.0001

# Initial guesses for joint angles
pi = np.pi
thetalist0 = np.array([3*pi/4, -pi/4, pi/2, 2*pi/3, -pi/4, pi/2])
print("\nInitial joint vector:", np.array2string(np.around(thetalist0, 4), separator=','), "\n")
print("Initial SE(3) end-effector config:\n", np.around(mr.FKinBody(M, Blist, \
                                                       thetalist0), 4), "\n")

# Add initial joint angles to csv file
writer.writerow(thetalist0)

# Newton Raphson iterations for joint angles
[theta_output, success] = IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev, writer)
print("Success:", success)
print("Final joint vectors:", np.array2string(np.around(theta_output, 4), separator=','))

# Close the csv file
f.close()