## Robot Kinematics - Week 3 Project
Modern Robotics - Mechanics, Planning, and Control Specialization

### Project Overview

The goal of this project is to modify the IKinBody function in the MR code library to print each iteration of the Newton-Raphson process, use this function to solve the inverse kinematics of the UR5 industrial robot, and visualize the convergence of the iterations with the CoppeliaSim robot simulator. The IKinBodyIterates function should also save the matrix of joint vectors as a .csv file, where each row of the text file consists of the comma separated joint values for that iterate.

### Code Overview

The [ik_iterations.py](https://github.com/saulakh/modern-robotics/blob/main/robot-kinematics/kinematics-project/ik_iterations.py) file includes the modified version of the IKinBody function from the MR code library, renamed to IKinBodyIterates. For this project, I included print statements for the joint vectors, end-effector configurations, error twists, and the angular and linear error magnitudes for each iteration. I saved each joint vector to a .csv file, which was used for the Coppelia simulation of the UR5 robot.

The dimensions of the UR5, the screw axes in the end-effector frame (B<sub>i</sub>), the home configuration of the end-effector (M), the desired configuration of the end-effector (T), and the error tolerances for the orientation (e<sub>ω</sub>) and linear position (e<sub>v</sub>) were provided. Using the joint sliders on the Coppelia simuator as a rough estimate, I set the initial joint angles as [3π/4, -π/4, π/2, 2π/3, -π/4, π/2].

The desired end-effector configuration is:

![image](https://user-images.githubusercontent.com/74683142/141325298-ac2fe78d-32c4-4fb2-ae2c-0f2d4091401d.png)

#### Newton-Raphson Iterations

Here is the output from running the ik_iterations.py file, which includes the initial joint angles and configuration, as well as the information for each iteration until convergence. This information is also saved in the [output_log.txt](https://github.com/saulakh/modern-robotics/blob/main/robot-kinematics/kinematics-project/output_log.txt) file:

```
Initial joint vector: [ 2.3562,-0.7854, 1.5708, 2.0944,-0.7854, 1.5708] 

Initial SE(3) end-effector config:
 [[-0.183  -0.017  -0.983  -0.5488]
 [ 0.183  -0.983  -0.017   0.3126]
 [-0.9659 -0.183   0.183   0.2191]
 [ 0.      0.      0.      1.    ]]


Iteration 1 :

Joint vector: [ 2.635 ,-0.5074, 1.3352, 2.1242, 0.5063, 1.7594]

SE(3) end-effector config:
 [[ 0.023   0.9997 -0.0079 -0.5946]
 [ 0.0912 -0.01   -0.9958  0.1232]
 [-0.9956  0.0222 -0.0914  0.0927]
 [ 0.      0.      0.      1.    ]]

Error twist V_b: [-0.0089 -0.0914 -0.0226 -0.0073  0.0948  0.0225]

Angular error magnitude: 0.0946

Linear error magnitude: 0.0977


Iteration 2 :

Joint vector: [ 2.5944,-0.6917, 1.7918, 2.0544, 0.5378, 1.556 ]

SE(3) end-effector config:
 [[-0.0039  0.9999 -0.0094 -0.4899]
 [-0.0066 -0.0094 -0.9999  0.0884]
 [-1.     -0.0038  0.0066  0.1063]
 [ 0.      0.      0.      1.    ]]

Error twist V_b: [-0.0094  0.0066  0.0038  0.0062 -0.0102 -0.0116]

Angular error magnitude: 0.0121

Linear error magnitude: 0.0166


Iteration 3 :

Joint vector: [ 2.5858,-0.6622, 1.7403, 2.0631, 0.5557, 1.5712]

SE(3) end-effector config:
 [[ 1.000e-04  1.000e+00 -1.000e-04 -4.997e-01]
 [ 2.000e-04 -1.000e-04 -1.000e+00  1.000e-01]
 [-1.000e+00  1.000e-04 -2.000e-04  9.990e-02]
 [ 0.000e+00  0.000e+00  0.000e+00  1.000e+00]] 

Error twist V_b: [-0.0001 -0.0002 -0.0001 -0.0001 -0.0003  0.    ]

Angular error magnitude: 0.0002

Linear error magnitude: 0.0003


Iteration 4 :

Joint vector: [ 2.5862,-0.6621, 1.7396, 2.064 , 0.5554, 1.5708]

SE(3) end-effector config:
 [[ 0.   1.  -0.  -0.5]
 [ 0.  -0.  -1.   0.1]
 [-1.   0.  -0.   0.1]
 [ 0.   0.   0.   1. ]]

Error twist V_b: [-0. -0. -0.  0. -0. -0.]

Angular error magnitude: 0.0

Linear error magnitude: 0.0

Success: True
Final joint vectors: [ 2.5862,-0.6621, 1.7396, 2.064 , 0.5554, 1.5708]
```

#### CSV File

Using the intial guess for the joint angles and the Newton-Raphson method, this function found the joint angles for the desired end-effector configuration in 4 iterations, and appended the values into a .csv file. Here are the joint vectors for each iteration, including the initial guess:

| Iteration  | θ<sub>1</sub> | θ<sub>2</sub> | θ<sub>3</sub> | θ<sub>4</sub> | θ<sub>5</sub> | θ<sub>6</sub> |
| ---------- | ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
|     0      | 2.35619449    | -0.785398163  |	 1.570796327	 | 2.094395102	  | -0.785398163	 | 1.570796327   |
|     1      | 2.635014764   | -0.507432588	 |  1.33516327	  | 2.124226581	  |  0.506290869	 | 1.759377041   |
|     2      | 2.594386447	  | -0.691720026	 |  1.791774692	 | 2.054399018	  |  0.537824705	 | 1.555954441   |
|     3      | 2.585847394	  | -0.662212528	 |  1.740310491	 | 2.063074048	  |  0.555694849	 | 1.571244664   |
|     4      | 2.586189173	  | -0.662084213	 |  1.73963464	  | 2.064042033	  |  0.555403431	 | 1.570796561   |

These values were stored in the [joint_vectors.csv](https://github.com/saulakh/modern-robotics/blob/main/robot-kinematics/kinematics-project/joint_vectors.csv) file, which was used in the Coppelia simulator for the UR5 robot.

### Project Results

Using the csv file and the Coppelia simulator, here is the configuration UR5 robot's end-effector configuration:

![image](https://user-images.githubusercontent.com/74683142/141223624-08b05d46-1696-475e-8649-27735a50fcd0.png)
