import numpy as np
from scipy.optimize import linprog

def stability_check(bodies, contacts):
    """
    Inputs:
    - bodies: n x 3 array (x,y of center of mass, and total mass)
    - contacts:
        - two bodies involved (0 is stationary ground) 
        - x,y location of contact
        - direction of contact normal (radians)
        - friction coefficient, mu
    Output:
    - stability: True if assembly will remain standing, False if not
    - k_list: List of k values from linprog function
    """
    m = len(bodies)
    n = len(contacts)
    # Bodies involved for each contact
    body_idxs = contacts[:,:2]

    for i in range(m):
        # List of contacts including this body
        c_col1 = np.where(body_idxs[:,0] == i+1)[0]
        c_col2 = np.where(body_idxs[:,1] == i+1)[0]
        c_idxs = np.concatenate((c_col1, c_col2))
        c = len(c_idxs)

        # Create matrix F for contact wrenches
        for j in range(c):
            body1, body2, x, y, theta, mu = contacts[c_idxs[j]]
            
            if i+1 == int(body2):
                theta = theta - np.pi
            
            # Friction cone angles
            alpha = np.arctan2(mu,1)
            angles = [theta + alpha, theta - alpha]

            for k in range(len(angles)):
                # Two friction cone edges per contact
                m_iz = x * np.sin(angles[k]) - y * np.cos(angles[k])
                F_i = np.array([m_iz, np.cos(angles[k]), np.sin(angles[k])])
                
                if j == 0 and k == 0:
                    F = F_i
                else:
                    F = np.vstack((F, F_i))
        
        # From Section 12.1.7.2
        f = np.ones((2*c,1))
        A = -1 * np.eye(2*c)
        b = -1 * np.ones((1,2*c))
        Aeq = F.T
        # b = -F_ext
        beq = np.array([9.81*bodies[i][0]*bodies[i][2], 0, 9.81*bodies[i][2]])
        # Find k such that Fk = b
        k_i = linprog(f,A,b,Aeq,beq)

        if i == 0:
            k_list = k_i.x
        else:
            k_list = np.concatenate((k_list, k_i.x))
        
        # If stability fails for either body, return false
        if k_i.success == False:
            return False, k_list

    if np.any(k_list < 0):
        return False, k_list

    return True, k_list

def main():
    # Bodies and contacts from project example (x_cm, y_cm, mass)
    bodies1 = np.array([[25, 35, 2], [66, 42, 5]])
    bodies2 = np.array([[25, 35, 2], [66, 42, 10]])

    # List of two bodies, contact x,y, direction of contact normal, friction coefficient
    contacts1 = np.array([[1,0,0,0,np.pi/2,0.1],[1,2,60,60,np.pi,0.5],[2,0,72,0,np.pi/2,0.5],[2,0,60,0,np.pi/2,0.5]])
    contacts2 = np.array([[1,0,0,0,np.pi/2,0.5],[1,2,60,60,np.pi,0.5],[2,0,72,0,np.pi/2,0.5],[2,0,60,0,np.pi/2,0.5]])

    # Print results of stability function
    key = {True: "remains standing", False: "collapses"}

    # From project examples (image 1)
    stability1, k1 = stability_check(bodies1, contacts1)
    print("\nAssembly 1:", key[stability1], "\nk:", np.around(k1,3))
    stability2, k2 = stability_check(bodies2, contacts2)
    print("\nAssembly 2:", key[stability2], "\nk:", np.around(k2,3))

    # Additional assembly (image 2)
    bodies3 = np.array([[10, 20, 3], [40, 20, 2]])
    bodies4 = np.array([[10, 20, 3], [40, 20, 2]])

    # List of two bodies, contact x,y, direction of contact normal, friction coefficient
    contacts3 = np.array([[1,0,0,0,np.pi/2,0.1],[1,0,20,0,np.pi/2,0.1],[1,2,20,30,np.pi,0.1],[2,0,60,0,np.pi/2,0.1]])
    contacts4 = np.array([[1,0,0,0,np.pi/2,0.6],[1,0,20,0,np.pi/2,0.6],[1,2,20,30,np.pi,0.6],[2,0,60,0,np.pi/2,0.6]])

    stability3, k3 = stability_check(bodies3, contacts3)
    print("\nAssembly 3:", key[stability3], "\nk:", np.around(k3,3))
    stability4, k4 = stability_check(bodies4, contacts4)
    print("\nAssembly 4:", key[stability4], "\nk:", np.around(k4,3))

    pass

if __name__ == "__main__":
    main()