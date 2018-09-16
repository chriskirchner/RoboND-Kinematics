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

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png
[DHTable]: ./misc_images/DHTable.PNG
[KR210_URDF]: ./misc_images/KR210_Annotations.PNG
[KR210_DH]: ./misc_images/KR210_DH.png
[WristCenter]: ./misc_images/WristCenter.png
[FellOver]: ./misc_images/KR210_FellOver.png
[CompletedProcess]: ./misc_images/CompletedProcess.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Rubric: Your writeup should contain a DH parameter table with proper notations and description about how you obtained the table. Make sure to use the modified DH parameters discussed in this lesson. Please add an annotated figure of the robot with proper link assignments and joint rotations (Example figure provided in the writeup template). It is strongly recommended that you use pen and paper to create this figure to get a better understanding of the robot kinematics.

I'm not sure why a description of how I obtained the DH table is being asked.  The table is provided in the KR210 Forward Kinematics 3 lecture. I obtained the table from this lecture.

To convert the URDF parameters to DH, the URDF parameters can be compiled into a table by extracting x, y, and z values for each joint.  For example, for "joint_1", z can be extracted from 'xyz=0 0 0.33' as '0.33' in the "origin" sub-tag.  Collecting the URDF parameters from each joint 1 through 6, including the gripper-joint, gives the URDF parameter table shown below, as provided in the KR210 Forward Kinematics 2 lecture.

![URDF Table][DHTable]

The links can then be constructed by connecting each joint to give the following annotated diagram of the KR210 arm, where the rotations follow the right-hand rule along the z-axis of each joint.

![KR210 Annotation][KR210_URDF]

Common normals are then assigned between each Z-axis as the X-axes.  Origins of each frame are made as the intersection of these normals with their bracketing Z-axes such that the most DH parameters are zero.  The following diagram was used to extract DH values from the URDF:

![KR210 DF Diagram][KR210_DH]


From the first frame {0}, the Z-axis is coincident with the axis of rotation for the first joint, making the x-axis of the first frame normal to this Z-axis.  Frame {0} is made coincident with frame {1} as their is no rotation needed in its zero configuration.  

Since the second joint is perpendicular to axis of rotation of the first joint, the X-axis of the second frame is made parallel to the first and the origin of the frame is at the intersection of joint_2, and not located at joint_1 to reduce the non-zero DH parameters.  The 90 degree left-hand rotation between Z0 and Z2 translates to -pi/2 for alpha1.  The distance between the two x-axes along the Z-axis for both initial frames is given as d1, or 0.75, which is the sum of the z-offsets for the joint_1 (0.33) and joint_2 (0.42) URDF parameters. 

Frame {2} at joint_2 has a x-offset of 0.35 to give n a1 of 0.35, the distance between z-axes along the x-axis.  As joint_3's axis of rotation is parallel to that of joint_2, the x-axis of frame {2} is made perpendicular to the x-axis of frame{1} to give a -pi/2 offset in theta_2. The negative is due to the negative rotation against the x-axis positive rotation, as defined as the right-hand rule along the z-axis.  

Frame {3} axis rotation mirrors that of frame {2} to give coincident x-axes.  The remaining frames have x-axes that are perpendicular to the x-axis of frame {2} as all remaining joints have z-axes in a plane that is normal to the x-axes. The only change in parameters between joint/frame {2} and {3} is a z-axis shift upward in the URDF parameters to give a d2 equal to 1.25, the distance between z-axes along the x-axis.  

The link to joint/frame {4} has an alpha3 based on the 90 degree left-hand rotation of the axis from joint_3 along the x-axis.  Its a3 is -0.54  based on the -0.054 z-shift in the URDF, which is the distance between the Z3 and Z4 axes along the X-axis.  The d4 parameter includes the x-shift of 0.96 between joint_3 and joint_4 as well as the 0.54 shift from joint_4 to joint_5 and relates to the x-shift for joint_4 and joint_5 in the URDF. As the z-axis is perpendicular in joint_3 to the z-axis in joint_4, and the di parameters is the distance between x-axes along the z-axis, the value for d4 includes the distance up to the next parallel z-axis on joint_5.

The link to joint/frame {5} has an alpha4 of pi/2 since the 90 degree rotation in z-axis is positive according to the right-hand rule of rotation along the joint_5 x-axis.  Joint {6} shows another 90 degree rotation in z-axis compared to that of joint {5}, but with a negative pi/2 based on the left-hand rotation along the x-axis.  The last frame is the gipper since this is the most meaningful origin to determine orientation and position of the robots hand.  There is only an URDF x-shift from joint_5 to the gripper to give a 0.303 dEE DH parameter.

The final DH table is shown below:

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 1.25 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Rubric: Your writeup should contain individual transform matrices about each joint using the DH table and a homogeneous transform matrix from base_link to gripper_link using only the position and orientation of the gripper_link. These matrices can be created using any software of your choice or hand written. Also include an explanation on how you created these matrices.

The following individual transform matrices were created using sympy, where the DH parameters were plugged in to the generic DH homogenous transformation as shown in the Denavit-Hartenberg Parameters lecture for the transformation between each link.  This generic matrix is derived by multiplications of the rotation matrix along the x-axis using the twist angle, the distance along the x-axis using the link length, the rotation along the z-axis using the joint angle, and the translation along the z-axis using the link offset.

```
## Individual transform matrices about each joint "j" to give Tji_ji+1


T0_1 = Matrix([
[cos(q1), -sin(q1), 0,    0],
[sin(q1),  cos(q1), 0,    0],
[      0,        0, 1, 0.75],
[      0,        0, 0,    1]])

T1_2 = Matrix([
[sin(q2),  cos(q2), 0, 0.35],
[      0,        0, 1,    0],
[cos(q2), -sin(q2), 0,    0],
[      0,        0, 0,    1]])

T2_3 = Matrix([
[cos(q3), -sin(q3), 0, 1.25],
[sin(q3),  cos(q3), 0,    0],
[      0,        0, 1,    0],
[      0,        0, 0,    1]])

T3_4 = Matrix([
[ cos(q4), -sin(q4), 0, -0.054],
[       0,        0, 1,    1.5],
[-sin(q4), -cos(q4), 0,      0],
[       0,        0, 0,      1]])

T4_5 = Matrix([
[cos(q5), -sin(q5),  0, 0],
[      0,        0, -1, 0],
[sin(q5),  cos(q5),  0, 0],
[      0,        0,  0, 1]])

T5_6 = Matrix([
[ cos(q6), -sin(q6), 0, 0],
[       0,        0, 1, 0],
[-sin(q6), -cos(q6), 0, 0],
[       0,        0, 0, 1]])

```
The generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose was obtained by pre-multiplying the individual rotation matrices for x-y-z axis rotation and adding the [x, y, z] position vector row-wise, followed by adding the [0,0,0,1] vector column-wise.  The configuration of the matrix is such that it can be multiplied with the a position vector of one frame to get transformation into new coordinates for another frame.

```

## Extrinsic RPY rotation of gripper link using world-view positioning and orientation

Rrpy = Matrix([
[cos(pitch)*cos(yaw), sin(pitch)*sin(roll)*cos(yaw) - sin(yaw)*cos(roll), sin(pitch)*cos(roll)*cos(yaw) + sin(roll)*sin(yaw), px],
[sin(yaw)*cos(pitch), sin(pitch)*sin(roll)*sin(yaw) + cos(roll)*cos(yaw), sin(pitch)*sin(yaw)*cos(roll) - sin(roll)*cos(yaw), py],
[        -sin(pitch),                               sin(roll)*cos(pitch),                               cos(pitch)*cos(roll), pz],
[                  0,                                                  0,                                                  0,  1]])

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Rubric: Based on the geometric Inverse Kinematics method described here, breakdown the IK problem into Position and Orientation problems. Derive the equations for individual joint angles. Your writeup must contain details about the steps you took to arrive at those equations. Add figures where necessary. If any given joint has multiple solutions, select the best solution and provide explanation about your choice (Hint: Observe the active robot workspace in this project and the fact that some joints have physical limits).

And here's where you can draw out and show your math for the derivation of your theta angles. 
            Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr[:3, :3]

# Calculate joint angles using Geometric IK method

## IK Position Problem

The wrist center position is derived by projecting the end-effector in z-axis onto the frame representing the end-effector's current orientation followed by subtracting the resulting position from the current position of the end-effector.  The multiplication of the rotation matrix used to re-orient the gripper provides the following set of equations for each axis, where w is the wrist position, p is the end-effector position, d+l is the length of the end effector, and n is the re-orientation vector from the z-axis.  

```python
wx = px - (d + l) * nx
wy = py - (d + l) * ny
wz = pz - (d + l) * nz
```


The following diagram did not make much sense to me as the coordinates used don't correspond to the KR210 joints in either the world or DH coordinates from lecture. 

![alt text][image2]

The derivation for the first three thetas determining wrist center position are depicted in the figure below:

![Wrist Center Thetas][WristCenter]

Theta1 is derived by applying atan2 to the Wx and Wy positions of the wrist center.  

``theta1 = atan2(wy, wx)``

Theta2 is derived by subtracting the angle 'a' and angle 'e' from pi/2 since the combination of angles makes a right triangle.  Angle a is derived using the law of cosines, while angle e is the atan2 of the height of the wrist center above the second joint, and the the length of the wrist arm from the second joint.  

```
c_side = s[a2]
b_side = sqrt(tx ** 2 + tz ** 2)
a_angle = acos((b_side ** 2 + c_side ** 2 - a_side ** 2) / (2 * b_side * c_side))
r = sqrt(wx ** 2 + wy ** 2)
tx = r - s[a1]
tz = wz - s[d1]
theta2 = (pi / 2 - a_angle - atan2(tz, tx))
```

Theta3 is derived by subtracting angle 'b' and adding angle 'f' from pi/2, since subtracting theta3 from pi equals pi/2 subtracted by angle 'b' and adding angle 'f'.  Theta3 and angle 'f' are subtracted since they are negative angles.

```
a_side = sqrt(s[d4] ** 2 + s[a3] ** 2)
b_angle = acos((a_side ** 2 + c_side ** 2 - b_side ** 2) / (2 * a_side * c_side)) 
f = atan2(s[a3], s[d4])
pi - theta3 = pi/2 - f + b
theta3 = pi/2 - b + f
theta3 = (pi / 2 - b_angle + f)
```

## IK Orientation Problem

The inverse kinematic problem can be solved by determining the rotation matrix for the wrist center using the angles derived in the inverse kinematic position problem, and multiplying the inverse of this matrix to Rrpy, the matrix representing the orientation of the end effector, to get the rotation matrix comprising of the last three thetas.  Comparing the generalized matrix of R3_6 to the valued matrix of R3_6 using the yaw, pitch, and roll values that ros provides gives the value of the thetas.

```
# Valued R3_6 Matrix
R0_3 = R01 * R12 * R23
R3_6 = R0_3.inv() * Rrpy

## Generalized R3_6 Matrix
R = R3_4 * R4_5 * R5_6 = Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
```

The atan2 functions are used in the derivation of the last three thetas to determine angles in all four quadrants of the euclidean plane. The equation for calculating each theta derives from reducing components of the R3_6 matrix to the tangent of the particular theta, then applying atan2 to get that angle.  For instance, to obtain theta5, dividing the magnitude of R13, -sin(q5)*cos(q4), and R23, sin(q4)*sin(q5), by R33, sin(q4)*sin(q5) reduces to atan2(tan(q5)) since cos^2 + sin^2 = 1.  

```
theta5 = atan2(R13/R33) 
= atan2(sqrt(-sin(q5)*cos(q4))^2 + sin(q4)*sin(q5)^2) / cos(q5))
= atan2(sqrt(sin(q5)^2*(cos(q4)^2 + sin(q4)^2))/cos(q5))
= atan2(sqrt(sin(q5)^2)/cos(q5))
= atan2(tan(q5))
```
Below are the following equations for each theta using the derivation process described above:

```
theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
theta5 = atan2(sqrt(R3_6[0, 2] ** 2 + R3_6[2, 2] ** 2), R3_6[1, 2])
theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
``` 

Ambiguous solutions are obtained for each angle, where theta5 is ambiguous as the reduced term is positive, where both X or Y are positive, or both are negative in atan2(X,Y).  The other angles are ambiguous since since the negative sign of the reduced term could apply to the X coordinate or Y coordinate in atan2(X,Y).  If X was negative, than the angle is positive, while if Y is negative, the angle is negative.  To select the best solution for the gripper, it is important that the gipper arm does not change rotation much with respect to the world coordinates, especially when the gripper is about to grip the object, since any tilting can cause the object to fall out of position.  To do this, the rotation of joint 4 is made opposite to joint 6 using the conditional below:

```
if sin(theta5) < 0:
    theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
    theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
else:
    theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
    theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Rubric: IK_server.py must contain properly commented code. The robot must track the planned trajectory and successfully complete pick and place operation. Your writeup must include explanation for the code and a discussion on the results, and a screenshot of the completed pick and place process.

Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  

The most challenging aspects of this coding assignment was understanding the use of linear algebra and its implication on the orientation and position of the KR210 arm.  It took much time understanding how to keep a level end effector with respect to the world coordinates, since any titling of the gripper just before gripping the object could cause the object to be displaced.  Looking at comments on slack helped resolve this issue by assigning opposite rotations to joint 4 and joint 6 based on the rotation of joint 5.  Another attempt to assign theta4 to theta6 values was based on the theta values that lead to a rotation matrix with the highest similarity to the rotation matrix built from yaw, pitch, and roll values from ros.  For some reason, the KR210 arm completely fell over:

![KR210 Falls Over][FellOver]

The second challenge was debugging the code and its eccentricities with the use of sympy.  For instance, the recommended use of inv("LU") led to high offsets between actual and intended end effector positions.  Using inv() resolved the offset issue.  There were also issues with over-writing rotation matrices in the trajectory for loop.  While rotations were'nt overwritten in the debug test script, the use of a loop caused over-writting which threw off the trajectory.  

The code can be improved with a deeper understanding of how to control the arm throughout the trajectory.  The current structure of the for loop assigns theta values in piecemeal based on individual positions along the trajectory.  Assigning thetas based on all points along the trajectory would leave to smoother more efficient rotations.

The final result of this project was a completed pick and place with 10/10.  However, previous attempts with other applications in the background caused weird bugs that caused the arm to jolt, loose the object mid-air, fall over at times, and not receive proper list trajectory point information.  A reboot and run of the pick and place process without other applications gave a successful 10/10.

![Completed Pick and Place Process][CompletedProcess]

