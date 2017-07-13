## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[relative_joint_locations]: ./misc_images/relJointLocations.jpg
[URDF_frame]: ./misc_images/URDF_frame.JPG
[world_frame]: ./misc_images/world_frame.JPG
[combine_frame]: ./misc_images/combined.jpg
[combine_frame1]: ./misc_images/combined1.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][relative_joint_locations]

Relative Position of joint<sub> i-1</sub> to joint <sub> i</sub> in meters (m). 

| joint | x | y | z | roll | pitch | yaw |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
joint<sub>1</sub> | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint<sub>2</sub> | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint<sub>3</sub> | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint<sub>4</sub> | 0.96 | 0 | -0.54 | 0 | 0 | 0 |
joint<sub>5</sub> | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint<sub>6</sub> | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper | 0.11 | 0 | 0 | 0 | 0 | 0 |

| joint | α<sub>i-1</sub> | ɑ<sub>i-1</sub> |ɗ<sub>i</sub> | θ |
| :---: | :---: | :---: | :---: | :---: |
joint<sub>1</sub> | 0 | 0 | 0.75 | q1 |
joint<sub>2</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | 0.35 | 0 | q2: q2 - <sup>π</sup>&frasl;<sub>2</sub> |
joint<sub>3</sub> | 0 | 1.25 | 0 | q3 |
joint<sub>4</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | -0.054 | 1.5 | q4 |
joint<sub>5</sub> | <sup>π</sup>&frasl;<sub>2</sub> | 0 | 0 | q5 |
joint<sub>6</sub> | <sup>-π</sup>&frasl;<sub>2</sub> | 0 | 0 | q7 |
gripper | 0 | 0 | 0.303 | q7: 0 |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.  

##### DH Parameter Table
```python
s = {alpha0:       0, a0:      0, d1:  0.75,
     alpha1: -pi / 2, a1:   0.35, d2:     0, q2: q2 - pi / 2,
     alpha2:       0, a2:   1.25, d3:     0,
     alpha3: -pi / 2, a3: -0.054, d4:   1.5,
     alpha4:  pi / 2, a4:      0, d5:     0,
     alpha5: -pi / 2, a5:      0, d6:     0,
     alpha6:       0, a6:      0, d7: 0.303, q7: 0}
```  

##### Individual Transformation Matrices About Each Joint
```python
# base_link to link_1
T0_1 = Matrix([[cos(q1), -sin(q1), 0, a0],
               [sin(q1) * cos(alpha0), cos(q1) * cos(alpha0), -sin(alpha0), -sin(alpha0) * d1],
               [sin(q1) * sin(alpha0), cos(q1) * sin(alpha0), cos(alpha0), cos(alpha0) * d1],
               [0, 0, 0, 1]])

# link_1 to link_2
T1_2 = Matrix([[cos(q2), -sin(q2), 0, a1],
               [sin(q2) * cos(alpha1), cos(q2) * cos(alpha1), -sin(alpha1), -sin(alpha1) * d2],
               [sin(q2) * sin(alpha1), cos(q2) * sin(alpha1), cos(alpha1), cos(alpha1) * d2],
               [0, 0, 0, 1]])

# link_2 to link_3
T2_3 = Matrix([[cos(q3), -sin(q3), 0, a2],
               [sin(q3) * cos(alpha2), cos(q3) * cos(alpha2), -sin(alpha2), -sin(alpha2) * d3],
               [sin(q3) * sin(alpha2), cos(q3) * sin(alpha2), cos(alpha2), cos(alpha2) * d3],
               [0, 0, 0, 1]])               

# link_3 to link_4
T3_4 = Matrix([[cos(q4), -sin(q4), 0, a3],
               [sin(q4) * cos(alpha3), cos(q4) * cos(alpha3), -sin(alpha3), -sin(alpha3) * d4],
               [sin(q4) * sin(alpha3), cos(q4) * sin(alpha3), cos(alpha3), cos(alpha3) * d4],
               [0, 0, 0, 1]])

# link_4 to link_5
T4_5 = Matrix([[cos(q5), -sin(q5), 0, a4],
               [sin(q5) * cos(alpha4), cos(q5) * cos(alpha4), -sin(alpha4), -sin(alpha4) * d5],
               [sin(q5) * sin(alpha4), cos(q5) * sin(alpha4), cos(alpha4), cos(alpha4) * d5],
               [0, 0, 0, 1]])

# link_5 to link_6
T5_6 = Matrix([[cos(q6), -sin(q6), 0, a5],
               [sin(q6) * cos(alpha5), cos(q6) * cos(alpha5), -sin(alpha5), -sin(alpha5) * d6],
               [sin(q6) * sin(alpha5), cos(q6) * sin(alpha5), cos(alpha5), cos(alpha5) * d6],
               [0, 0, 0, 1]])

# link_6 to gripper
T6_7 = Matrix([[cos(q7), -sin(q7), 0, a6],
               [sin(q7) * cos(alpha6), cos(q7) * cos(alpha6), -sin(alpha6), -sin(alpha6) * d7],
               [sin(q7) * sin(alpha6), cos(q7) * sin(alpha6), cos(alpha6), cos(alpha6) * d7],
               [0, 0, 0, 1]])
```

##### Generalized homogeneous transform between base_link and gripper using only the gripper pose.
```python
R_corr = rot_z(pi) * rot_y(-pi / 2)
T0_2 = simplify(T0_1 * T1_2)
T0_3 = simplify(T0_2 * T2_3)
T0_4 = simplify(T0_3 * T3_4)
T0_5 = simplify(T0_4 * T4_5)
T0_6 = simplify(T0_5 * T5_6)
T0_G = simplify(T0_6 * T6_7)
T_total = T0_G * R_corr
```

As shown below the URDF frame is different from our world frame so we need to apply a correction rotation matrix in order to correct our transformation matrix.
![alt text][combine_frame]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.




### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


