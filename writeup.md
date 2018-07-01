## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[robot_figure]: ./misc_images/robot_figure.png
[DH_table]: ./misc_images/DH_table.png
[DH_matrix]: ./misc_images/DH_transform_matrix.png
[base_gripper_transform]: ./misc_images/base_gripper_transform.png
[q1_derivation]: ./misc_images/q1_derivation.png
[q2_derivation]: ./misc_images/q2_derivation.png
[q3_derivation]: ./misc_images/q3_derivation.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

This file.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

With the robot shown in the simulation, the joints are identified.

A robot figure in DH convention is constructed.

![robot figure][robot_figure]

The DH parameter table is written using the above figure.

![DH parameters table][DH_table]

By reading [kr210.urdf.xacro](./kuka_arm/urdf/kr210.urdf.xacro), the respective numerical values for parameters can be found:
* a1 = 0.35
* a2 = 1.25
* a3 = 0.054
* d1 = 0.33 + 0.42
* d4 = 0.96 + 0.54
* dg = 0.193 + 0.11

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The transform matrices about each joint are created by filling the values into the DH transform matrix as shown below.

![DH transform matrix][DH_matrix]

The homogeneous transform matrix between base_link and gripper_link can also be constructed from gripper_link's orientation and position.

Because the orientation of the gripper is given as extrinsic roll, pitch, yaw angles, the rotation matrix from base to gripper can be constructed as:    
R = Rz(yaw) * Ry(pitch) * Rx(roll)    
where Rz, Ry and Rx are rotation matrices around respective axes.

A caveat is that the angles given are in URDF model's gripper reference frame, which does not align with DH reference frame shown in previous robot figure.    
Therefore, a correction matrix needs to be constructed.

Because URDF gripper frame can be obtained from DH frame by intrinsic rotation first around Z-axis for pi, and then Y-axis for -pi/2, Rcorr can be computed as:    
Rcorr = Rz(pi) * Ry(-pi/2)

Therefore, rotation expressed in DH gripper reference frame is:    
Rrpy = R * Inverse(Rcorr)

The position of the gripper is also given, and can be put into vector pee.

The homogeneous transform matrix from base to gripper is therefore:

![base gripper transform][base_gripper_transform]

Written in symbolic form:

sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw) | -sin(pitch)*sin(roll)*cos(yaw) + sin(yaw)*cos(roll) | cos(pitch)*cos(yaw) | x
sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw) | -sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw) | sin(yaw)*cos(pitch) | y
cos(pitch)*cos(roll) | -sin(roll)*cos(pitch) | -sin(pitch) | z
0 |  0 | 0 | 1

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Because the rotation axes of joint4, joint5 and joint6 intersect at a single point, they form a spherical wrist, where the intersection point is wrist center.

The Inverse Kinematics problem can therefore be decomposed into position and orientation.

##### Position
The target position of wrist center is target position of end-effector minus dg along Z6, as shown in the robot figure:    
pwc_target = pee - dg * Rrpy[:, 2]

Because q1 determines the relative values of x and y, as in the below top-down view:

![q1 derivation figure][q1_derivation]

**q1 = atan2(y, x)**

To find q2:

![q2 derivation figure][q2_derivation]

* h = z - d1   
* l = sqrt(x^2 + y^2) - a1
* phi = atan2(h, l)
* A = sqrt(a3^2 + d4^2)
* B = a2
* C = sqrt(h^2 + l^2)
Using law of cosines, get
* a = acos((B * B + C * C - A * A) / (2 * B * C))

Finally,    
**q2 = pi/2 - a - phi**

To find q3:

![q3 derivation figure][q3_derivation]

* rho = atan2(a3, d4) + pi/2
Again using law of cosines, get
* c = acos((B * B + A * A - C * C) / (2 * B * A))

Which gives:
**q3 = pi - c - rho**

##### Orientation
To find, angles for last 3 joints, we first find the rotation matrix from joint3 to joint6
R3_6 = R3_g = Inverse(R0_3) * Rrpy

It can be computed to be:

-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6) | -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5) | -sin(q5)*cos(q4)
sin(q5)*cos(q6) | -sin(q5)*sin(q6) | cos(q5)
-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4) | sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6) | sin(q4)*sin(q5)

Define p = sqrt(r13 * r13 + r33 * r33), then
**q5 = atan2(p, r23)**    
**q4 = atan2(r33 / p, -r13 / p)**
**q6 = atan2(-r22 / p, r21 / p)**

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The model parameters are initialized using values read from xacro file, and the forward kinematics matrices are constructed.

For each requested pose, the orientation and position are extracted. The homogeneous transformation matrix between base and gripper can be calculated.    
From the gripper target position and transformation matrix, the target position of wrist center can be calculated as discussed above, which gives first three joint angles.

From the gripper target orientation, the last three joint angles can also be calculated as above.

The results are then assembled and returned.

A screen recording of execution can be found at [misc_videos/video.mp4](./misc_videos/video.mp4)