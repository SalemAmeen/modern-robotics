import modern_robotics as mr
import numpy as np

# Problem 1
def newton_raphson(x0,f,df,n):
    # Initial value
    x = x0
    
    # Find value after n iterations
    for i in range(n):
        x = x - f(x)/df(x)
    
    return x

fx = lambda x: x**2 - 9
dfx = lambda x: 2*x
fy = lambda y: y**2 - 4
dfy = lambda y: 2*y

x2 = newton_raphson(1, fx, dfx, 2)
y2 = newton_raphson(1, fy, dfy, 2)
print("\nProblem 1: \n", [x2, y2])

# Problem 2
Slist = np.array([[0,0,1,0,0,0],
                [0,0,1,0,-1,0],
                [0,0,1,0,-2,0]])
M = np.array([[1,0,0,3],
            [0,1,0,0],
            [0,0,1,0],
            [0,0,0,1]])
T = np.array([[-0.585,-0.811,0,0.076],
            [0.811,-0.585,0,2.608],
            [0,0,1,0],
            [0,0,0,1]])
thetalist0 = np.array([0.7854,0.7854,0.7854])
eomg = 0.001
ev = 0.0001
[thetalist, success] = mr.IKinSpace(np.transpose(Slist),M,T,thetalist0,eomg,ev)

print("\nProblem 2:\n", np.array2string(thetalist, separator=','))