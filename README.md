[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)

## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/urdf&#32;frame.png
[image2]: ./misc_images/urdf.png
[image3]: ./misc_images/degree&#32;offset.png
[image4]: ./misc_images/griffer&#32;frame.png
[image6]: ./misc_images/dh&#32;parameter.png
[image7]: ./misc_images/calculate_theta.jpeg
[image8]: ./misc_images/result.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Robot parameters are described in `kr210.urdf.xacro`. It has all the same reference frame on all the links. (R:x, G:y, B:z)
![alt text][image1]
Each joint has the `origin` position from the parent link and `rpy` orientation link and rotation `axis`. We can derive DH parameter from this table.
```
<joint name="joint_6" type="revolute">
    <origin xyz="0.193 0 0" rpy="0 0 0"/>
    <parent link="link_5"/>
    <child link="link_6"/>
    <axis xyz="1 0 0"/>
    <limit lower="${-350*deg}" upper="${350*deg}" effort="300" velocity="${219*deg}"/>
</joint>
<joint name="gripper_joint" type="fixed">
    <parent link="link_6"/>
    <child link="gripper_link"/>
    <origin xyz="0.11 0 0" rpy="0 0 0"/><!--0.087-->
    <axis xyz="0 1 0" />
</joint>
```
In URDF file, each joint from joint1 to joint6 has rotation `axis` like theses direction, {1:z, 2:y, 3:y, 4:x, 5:y, 6:x}. We set that as `Zi` and each `Xi-1` as normal for both `Zi-1 and Zi`. 
![alt text][image2]

And then we can derive DH parameters. We use DH parameter definition from `Craig, JJ. (2005). Introduction to Robotics: Mechanics and Control, 3rd Ed (Pearson Education, Inc., NJ)`.
* `Twist angle alphai-1` is the angle `from Zi-1 to Zi`. 
* `Link length ai-1` is the distance `from Zi-1 to Zi` along `Xi-1`.
* `Link offset di` is the distance `from Xi-1 to Xi` along `Zi`.
* `Joint angle thetai` is the angle `from Xi-1 to Xi`.

Joint4,5,6 become `spherical wrist` and Joint5 becomes `wrist center` and for convenience, we define these `Origin4,5,6 at the same position` and Xg as parallel to X4,5,6. Here Xg gets different to URDF direction. 
![alt text][image6]
```
# Create Modified DH parameters
s = {alpha0:      0, a0:      0, d1:     0.75,  q1:         q1,
        alpha1: -pi/2., a1:   0.35, d2:        0,  q2:    q2-pi/2,
        alpha2:      0, a2:   1.25, d3:        0,  q3:         q3,
        alpha3: -pi/2., a3: -0.054, d4:      1.5,  q4:         q4,
        alpha4:  pi/2., a4:      0, d5:        0,  q5:         q5,
        alpha5: -pi/2., a5:      0, d6:        0,  q6:         q6,
        alpha6:      0, a6:      0, d7:    0.303,  q7:          0}
```
#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 | - pi/2 | - 0.054 | 1.5 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | - pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

Using this DH parameters, I make total transform composed of homogenious transform TF from each joint.
```
TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                    [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                    [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                    [                0,                 0,           0,             1]])
```
```
T0_ee = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_ee
```
The URDF file is using the different local frame at the gripper. So multiply another transform for it.
![alt text][image4]
```
Ree_rviz_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2.)
R0_rviz = R0_ee * Ree_rviz_corr
R0_ee = R0_rviz * Ree_rviz_corr.transpose()
```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
For inverse kinematics, we should use numerical or analyical approach.
* Numerical approach: There is no guarantee that the algorithm will converge or do so quickly enough to meet application requirements and it only returns one solution. So to generate solutions for the various possible poses, different initial conditions must be used.
* Analytical approach: Only certain types of manipulators are solvable in closed-form. If either of the following two conditions are satisfied, then the serial manipulator is solvable in closed-form.
    * Three neighboring joint axes intersect at a single point, or
    * Three neighboring joint axes are parallel (which is technically a special case of 1, since parallel lines intersect at infinity)

Kr210 has wrist center and we can analytical approach. we kinematically decouple the position and orientation of the end effector. Instead of solving twelve nonlinear equations simultaneously (one equation for each term in the first three rows of the overall homogeneous transform matrix), it is now possible to independently solve two simpler problems: 
1. first, the Cartesian coordinates of the wrist center, 
    * If we choose Z5 parallel to ZEE and pointing from the WC to the EE, then this displacement is a simple translation along ZEE. The magnitude of this displacement, let’s call it d,
    * `0r_wc = 0r_rviz - 0r_rviz/wc`
        * = 0r_rviz - d * R0_rviz * rviz_[1,0,0].T
        * = 0r_rviz - d * R0_ee * Ree_rviz(corr) * rviz_[1,0,0].T
        * = 0r_rviz - d * R0_ee * ee_[0,0,1].T
    * `0r_wc = 0r_ee - d * R0_ee * ee_[0,0,1].T`
        * `= 0r_ee - d * R0_ee[:,2]`
        * `0r_ee` is the position of EE expressed in the base frame is given by EE pose.position.
        * `R0_rviz_rpy` is the relative orientaion of EE from base frame and given by EE pose.orientation. R0_rviz_rpy = R_z * R_y * R_x
        * `R0_rviz = R0_ee * Ree_rviz_corr = R0_rviz_rpy`. 
        * So `R0_ee = R0_rviz_rpy * Ree_rviz_corr.transpose()`
    * find theta1,2,3 before WC joint by comparing each term of both sides.
    ![alt text][image7]

2. and then the composition of rotations to orient the end effector. 
    * `R3_6 = R0_3.transpose() * R0_ee`
    * R3_6 = Matrix([
        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)], 
        [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)], 
        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5)]])
    * By comparing each term of both sides,
        * theta4 = atan2(R3_6[2,2], -R3_6[0,2])
        * theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
        * theta6 = atan2(-R3_6[1,1], R3_6[1,0])


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The important part is the meaning of the `req.pose.position` and `req.pose.orientation`. 
* `req.pose.postion` presents the position of EE expressed in the base frame.
    * `0r_ee = [req.pose.position.x, req.pose.position.y, req.pose.position.z].T`
* `req.pose.orientation` presents the relative orientation of EE from base frame.
    * `R0_rviz = R_yaw * R_pitch * R_roll`

The subtle problem of `sympy` is that the result of `subs()` and `evalf()` can be different. `subs()` seems to postpone `pi` until it can. So `subs()` gave more precise result.

I got 10/10! `ROS` is running on `mac` natively compiled!
![alt text][image8]










---
# Robotic arm - Pick & Place project

Make sure you are using robo-nd VM or have Ubuntu+ROS installed locally.

### One time Gazebo setup step:
Check the version of gazebo installed on your system using a terminal:
```sh
$ gazebo --version
```
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
$ git clone https://github.com/udacity/RoboND-Kinematics-Project.git
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

For demo mode make sure the **demo** flag is set to _"true"_ in `inverse_kinematics.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch

In addition, you can also control the spawn location of the target object in the shelf. To do this, modify the **spawn_location** argument in `target_description.launch` file under /RoboND-Kinematics-Project/kuka_arm/launch. 0-9 are valid values for spawn_location with 0 being random mode.

You can launch the project by
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ ./safe_spawner.sh
```

If you are running in demo mode, this is all you need. To run your own Inverse Kinematics code change the **demo** flag described above to _"false"_ and run your code (once the project has successfully loaded) by:
```sh
$ cd ~/catkin_ws/src/RoboND-Kinematics-Project/kuka_arm/scripts
$ rosrun kuka_arm IK_server.py
```
Once Gazebo and rviz are up and running, make sure you see following in the gazebo world:

	- Robot
	
	- Shelf
	
	- Blue cylindrical target in one of the shelves
	
	- Dropbox right next to the robot
	

If any of these items are missing, report as an issue.

Once all these items are confirmed, open rviz window, hit Next button.

To view the complete demo keep hitting Next after previous action is completed successfully. 

Since debugging is enabled, you should be able to see diagnostic output on various terminals that have popped up.

The demo ends when the robot arm reaches at the top of the drop location. 

There is no loopback implemented yet, so you need to close all the terminal windows in order to restart.

In case the demo fails, close all three terminal windows and rerun the script.

