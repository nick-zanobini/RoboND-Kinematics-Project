Project: Kinematics Pick & Place
---
The video of my code in action can be seen [here](https://youtu.be/JGvfU6yCrSk)  
---

To run projects from this repository you need version 7.7.0+
If your gazebo version is not 7.7.0+, perform the update as follows:
```sh
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install gazebo7
```

Once again check if the correct version was installed:
```sh
$ gazebo --version
```
### For the rest of this setup, catkin_ws is the name of active ROS Workspace, if your workspace name is different, change the commands accordingly

If you do not have an active ROS workspace, you can create one by:
```sh
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

Now that you have a workspace, clone or download this repo into the **src** directory of your workspace:
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/nick-zanobini/RoboND-Kinematics-Project.git
```

Now from a terminal window:

```sh
$ cd ~/catkin_ws
$ rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ sudo chmod +x target_spawn.py
$ sudo chmod +x IK_server.py
$ sudo chmod +x safe_spawner.sh
```
Build the project:
```sh
$ cd ~/catkin_ws
$ catkin_make
```

Add following to your .bashrc file
```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/models

source ~/catkin_ws/devel/setup.bash
```

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

    - Robot
    
    - Shelf
    
    - Blue cylindrical target in one of the shelves
    
    - Dropbox right next to the robot


**Steps to complete the project:**  
1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[relative_joint_locations]: ./misc_images/relJointLocations.jpg
[URDF_frame]: ./misc_images/URDF_frame.jpg
[world_frame]: ./misc_images/world_frame.JPG
[combine_frame]: ./misc_images/combined.jpg
[combine_frame1]: ./misc_images/combined1.jpg
[forward_demo]: ./misc_images/forward_demo.jpg
[theta2_img]: ./misc_images/theta2.jpg
[theta3_img]: ./misc_images/theta3.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

`roslaunch kuka_arm forward_kinematics.launch`:
![alt text][forward_demo]

The DH parameters were derived from the arm according to these axis assignments:  
![alt text][relative_joint_locations]  

##### Table 1: Relative Position of joint<sub>i-1</sub> to joint <sub>i</sub> in meters (m).  


| joint | x | y | z | roll | pitch | yaw |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: |
joint<sub>1</sub> | 0 | 0 | 0.33 | 0 | 0 | 0 |
joint<sub>2</sub> | 0.35 | 0 | 0.42 | 0 | 0 | 0 |
joint<sub>3</sub> | 0 | 0 | 1.25 | 0 | 0 | 0 |
joint<sub>4</sub> | 0.96 | 0 | -0.54 | 0 | 0 | 0 |
joint<sub>5</sub> | 0.54 | 0 | 0 | 0 | 0 | 0 |
joint<sub>6</sub> | 0.193 | 0 | 0 | 0 | 0 | 0 |
gripper | 0.11 | 0 | 0 | 0 | 0 | 0 |  


##### Table 2: Modified DH Parameters
α<sub>i-1</sub>: twist angle, angle between Ζ<sub>i-1</sub> and Ζ measured about Χ<sub>i-1</sub>  
ɑ<sub>i-1</sub>: link length, distance from Ζ<sub>i-1</sub> and Ζ measured about Χ<sub>i-1</sub>  
ɗ<sub>i</sub>: link offset, signed distance from Χ<sub>i-1</sub> to Χ<sub>i</sub> measured along Ζ<sub>i</sub>  
θ<sub>i</sub>: joing angle, angle between Χ<sub>i-1</sub> to Χ<sub>i</sub>

| joint | α<sub>i-1</sub> | ɑ<sub>i-1</sub> |ɗ<sub>i</sub> | θ<sub>i</sub> |
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

Inverse Kinematics: `R = R0_3 * R3_6`  
Inverse Position Kinematics: `R0_3`  : 0<sub>1</sub>, 0<sub>2</sub>, 0<sub>3</sub>  
Inverse Orientation Kinematics: `R3_6 = R0_3.inv() * R0_6` : 0<sub>4</sub>, 0<sub>5</sub>, 0<sub>6</sub>

Wrist Center: `P_EE - d7 * R0_6 * [0;0;1]`  


 
### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

Gripper Position: `P_EE = Matrix([[px, py, pz]])`  
Wrist Center Position: `P_WC` is a matrix with the format `[pwc_x, pwc_y, p_wc_z]`  
The function N() is used to evaluate pi throughout the program.
1. Find the position of the wrist center

    Given the position of the gripper we can find the position of the wrist center: `P_WC = simplify(P_EE - (0.303) * R0_6 * Matrix([[0], [0], [1]]))`

1. Find θ<sub>1</sub>

     ```python
     theta1 = N(atan2(P_WC[1], P_WC[0]))
     ```
![alt text][theta3_img]  
1. Find Find θ<sub>3</sub>
    ```python
    # Calculate position of joint 3
    P0_2 = P_2_sym.evalf(subs={q1: theta1})
    x_c, y_c, z_c = P_WC - P0_2[0:3, :]
    ```
    ```python
    # link lengths
    l_23 = a2
    l_35 = sqrt(a3**2 + d4**2)
    l_25 = sqrt(x_c**2 + y_c**2 + z_c**2)
    ```
    ```python
    # Intermediate angles for theta3 and theta3
    D1 = (l_25**2 - l_23**2 - l_35**2) / (2 * l_23 * l_35)
    theta_31 = atan2(a3, d4)
    theta_32 = atan2(sqrt(1 - D1**2), D1)
    theta3 = N((theta_32 + theta_31 - (pi / 2)).subs(s))
    ```
![alt text][theta2_img]  
1. Find Find θ<sub>2</sub>
    ```python
    # Calculate theta2
    D2 = (l_23**2 - l_35**2 + l_25**2) / (2 * l_23 * l_25)
    theta_21 = atan2(sqrt(1 - D2**2), D2)
    theta_22 = atan2(z_c, sqrt(x_c**2 + y_c**2))
    theta2 = N(((pi / 2) - theta_21 - theta_22).subs(s))
    ```

1. Find Find θ<sub>4</sub>, θ<sub>5</sub> and θ<sub>6</sub>
    ```python
    # Rotation Matrices for theta4 theta5 and theta6 calculations
    R_corr = rot_z(pi) * rot_y(-pi / 2)
    R0_6 = R0_6[0:3, 0:3] * R_corr
    R0_3_inv_sym = simplify((T0_3[0:3, 0:3]).inv())
    R0_3_inv = R0_3_inv_sym.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
    R3_6 = np.array(R0_3_inv * R0_6).astype(np.float64)
    ```
    ```python
    # Individual Rotation Matrix elements for theta4 theta5 and theta6 calculations
    r12, r13 = R3_6[0, 1], R3_6[0, 2]
    r21, r22, r23 = R3_6[1, 0], R3_6[1, 1], R3_6[1, 2]
    r32, r33 = R3_6[2, 1], R3_6[2, 2]
    ```
    ```python
    # Calculate theta4, theta5 and theta6
    if np.abs(r23) is not 1:
        theta5 = atan2(sqrt(r13**2 + r33**2), r23)
         # workaround for smooth movement
        if sin(theta5) < 0:
            theta4 = atan2(-r33, r13)
            theta6 = atan2(r22, -r21)
        else:
            theta4 = atan2(r33, -r13)
            theta6 = atan2(-r22, r21)
    else:
        if r23 == 1:
            theta5 = 0
            theta4 = -theta6 + atan2(-r12, -r32)
        else:
            theta5 = 0
            theta4 = theta6 - atan2(r12, -r32)
    ```

