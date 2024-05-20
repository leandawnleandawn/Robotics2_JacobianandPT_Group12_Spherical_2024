# Final Project: Path and Trajectory Planning

## Abstract

## Introduction of the Project

  After the following use of Forward and Inverse Kinematics of a given Manipulator, we expand our horizons by
  learning the other aspects of the kinematics: Velocity and Acceleration. If the Forward Kinematics of the Manipulator
  is based on what the Denavit Hartenberg Notations that created Homogenous Transformation Matrices and Inverse Kinematics is 
  focused on either the graphical method or numerical solutions (as other textbooks indicated), Jacobian Matrix uses the convention of
  what Homogenous Transformation Matrices derived on the Forward Kinematics and used as basis for the generation of the Matrix itself.
  
  As Spong et. al. (1989) defined, The Jacobian Matrix denotes as the vector version on what the Transformation Matrices that created in the 
  Forward Kinematics. This is important as Spong et. al. (1989) emphasizes that it is the heart on what smooth transistion between positions and
  orientations of the end effectors between points takes place. Corke (2023) explained on how the Jacobian Matrix can be translated into the equation:

  $$\dot v = J \dot q$$

  This is a 6 x n matrix where n is the number of joints in a given manipulator system. 

  The power of the Jacobian can be seen in finding the Robot Singularities of the System, Inverse Velocity Kinematics using the Inverse Jacobian, and 
  Forces and Torque Analysis of the Manipulator by its Transposed matrix. Once accomplished, we can infer on how the manipulator can move between two set of points via 
  Path and Trajectory

  The proponents of this repository proposed to create a Graphical User Interface of on Integrating the Forward, Inverse, Differential Kinematics and its Path and Trajectory
  Planning. 
  

## Jacobian Matrix of Spherical Manipulator

  Since Corke (2023) defined the Jacobian as a equation:

  $$\dot v = J \dot q$$

  There are two special cases to consider on a manipulator based on what joint it is having. 

  Take note on the previous lectures that a 
  
### Case 1 : Prismatic Joint
 In Spong (1989) emphasized, a prismatic joint only moves on the translation axis of the end effector. With this case it can be written as:

 ![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/753c6c4e-7e22-4da8-bcf0-e8853dea2d43)\
![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/bc45caad-b0f1-453d-b495-756fb85c9d0c)\
![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/a5e70e62-0bba-4808-b8a6-92ca9fcd0618)\
![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/846121eb-fccc-41e3-a6c1-d69c82e739c9)

This means that the robot is dependent on the Rotation matrices and the Position Vector that it is having. 

Applying a partial derivative to itself. 

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/6356b3e4-8916-4731-924a-6139f8d47539)\
![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/b9e66075-ca21-4192-807e-1d4b5ecea70c)\
![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/91d9c1c8-63b0-41a6-b82a-9a00a226ac6a)

Excluding the following time derivative of joint d, we get the prismatic joint value of the Jacobian as:

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/135568b1-ced8-4ba1-bae0-0384480fe1b9)

This is defined as $z_{i-1}$

### Case 2 : Revolute Joint

In Revolute Joint, 

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/de734e4f-56d7-4575-8d6b-039be2f12877)

Computing this Partial Derivatives comes a equation:

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/66d34417-1151-4297-869d-a7734e31d0bc)

For the First term then:

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/a0a6a7f2-13e6-4f85-ae88-170f8fd8e3fd)

Via straightforward computation by Spong et. al. (1989). The Revolute Joint is the following:

![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/71f62b74-c39d-4aed-a882-5fd6851cdef2)


In this video, the proponents tackled the following Jacobian Matrix given itself. 


## Differential Equation of Spherical Manipulator

Based on the Corke (2023) equation:
  $$\dot v = J \dot q$$

  It can be expanded into:

  ![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/cdef6f3b-523a-47e4-948e-050346116a99)\

  ![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/8487a8b6-b0b6-4799-9149-a4f6f11c8dc0)

  It can generate the following Differential Equation as Following:

  ![image](https://github.com/leandawnleandawn/Robotics2_JacobianandPT_Group12_Spherical_2024/assets/83767299/756e7fd4-85c8-4441-923b-df8edb17a1c2)

  This Video explores how the following Jaocbian Matrix can be interpreted as the Differential Equation of the manipulator
  

## (Additional) Robot Singularities and Inverse Jaocbian

  Once the Jacobian Matrix and the Differential Equation is defined, the following Singularities can be computed. This is important on which part of the Manipulator
  that will not reach its certain positions, this is computed as Spong et. al. (1989) defined as on 6x6 matrix and a configuration q:

  $$det(J(q)) = 0$$

  In this Jacobian Matrix, the first three row of the manipulator emphasizes the position of the manipulator. That is the part that will take the Singularity. On the last
  three rows of the manipulator it is always defined as a singularity itself. 

  Methods such as Manual Computation or Numerical Analysis are acceptable for this case. 

  Once the Determinant of the Jacobian at given joint value q is zero, it reached its own Singularity. Meaning that one of the joint of the manipulator tend not exist that period
  lessening the following number of joints in the system.

  If the Singularity doesn't exist, then the Robot can move at those positions.


  In Inverse Jacobian, it is equivalent on Inverse Kinematics wherein the given end effector velocities gave the differential equations of the joint variable velocitie of 
  given manipulator or:

  $$\dot q = J^{-1} \dot v$$

  Similar to Jacobian Matrix, techniques such as finding the 
  
## (Additional) Torques and Forces Analysis


## Path and Trajectory Planning of Spherical Manipulator


## References

Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2005). Robot modeling and control. John Wiley & Sons.
Corke, P. (2023). Robotics, vision and control: Fundamental Algorithms in Python. Springer.
