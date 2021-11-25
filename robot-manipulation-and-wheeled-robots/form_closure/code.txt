import numpy as np
from scipy.optimize import linprog

def form_closure_check(contacts):
    """
    Inputs:
    - contacts: n x 3 array (x,y,theta)
    - x,y: coordinates of stationary point contacts
    - theta: direction of contact normal, in radians
    Output:
    - form_closure: True if body is in form closure, False if not
    """

    j = len(contacts)

    # For planar bodies, min 4 point contacts needed for form closure
    if j < 4:
        return False

    # Create matrix F for contact wrenches
    for i in range(j):
        x = contacts[i,0]
        y = contacts[i,1]
        theta = contacts[i,2]

        m_iz = int(x * np.sin(theta) - y * np.cos(theta))
        F_i = np.array([m_iz, int(np.cos(theta)), int(np.sin(theta))])

        if i == 0:
            F = F_i
        else:
            F = np.vstack((F, F_i))

    F = F.T
    
    # From Section 12.1.7.2
    f = np.ones((1,j))
    A = np.array([[-1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,-1]])
    b = -1 * np.ones((1,j))
    Aeq = F
    beq = np.array([0,0,0])
    
    # Find k such that Fk = 0
    k = linprog(f,A,b,Aeq,beq)

    return k.success, k.x

# Contact x,y coordinates, and directions of contact normals
contacts1 = np.array([[0,1,np.pi],[0,1,np.pi/2],[2,0,0],[2,0,3*np.pi/2]])
contacts2 = np.array([[0,1,np.pi],[0,1,np.pi/2],[2,1,0],[2,1,np.pi/2]])

# Print results of form closure function
success1, k1 = form_closure_check(contacts1)
success2, k2 = form_closure_check(contacts2)

print("\nForm closure for image 1:", success1)
if success1 == True:
    print("k:", k1)

print("\nForm closure for image 2:", success2)
if success2 == True:
    print("k:", k2)