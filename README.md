# Assignment 1: Inverse Kinematics Using Optimization
Thu Nguyen & Alexander Volkov

## Files
To download all files as a zip, click on the green "Clone or download" button on the right, and then "Download ZIP" in the dropdown menu. Note, if the "link_length" vector is changed (different number of links or different lengths), symbolicFunctionGeneration(link_length) must be re-executed to solve for the new forward kinematics symbolic equations. Anything about n=2 links takes quite a while to generate, so be patient... it gets there (: 

- **runMe.m**: The main assignment file. This can be executed to generate all results for this assignment.
- **IKcost.m**: This function implements the cost function used by fmincon (**part1.m** **part2.m**) to solve for IK.
- **updateSnakeBotDrawing.m**: Refreshes 3D snake robot graphics.
- **symbolicFunctionGeneration.m** Function to generate forward kinematics functions using symbolic expressions.
- **part1.m**: Solution to part 1 of the assignment, using fmincon to solve for IK on the snake robot, with hard constraints for the joint limits and obstacle collisions. Cost function gradient and hessian not used here. Graphics are refreshed during runtime to show incremental optimization results.
- **part2.m**: Solution to part 2 of the assignment. Virtually the same as part 1, except for the use of a cost function gradient by the fmincon optimizer.
- **part3.m**: Solution to part 3 of the assignment. Virtually the same as part 2, except for the use of different optimization algorithms.
- **part4.m**: Solution to part 4 of the assignment. Virtually the same as part 1, except repeats optimization for a variety of randomly sampled initial conditions, in order to (hopefully) find a variety of locally optimal solutions.

## Overview
Traditional inverse kinematics (IK) solvers for serial link mechanisms are implemented using lookup tables or explicit analytically-derived functions. Advances in symbolic equation solvers and optimization algorithms enable an alternative, more automated and less cognitively involved approach. In this assignment, a 3D n-link serial chain IK solver (and object collision avoidance) is treated as a constrained optimization problem, implemented using MATLAB's "fmincon" function and the "Symbolic Toolbox" to automatically derive the forward kinematics (FK) for an arbitrary n-link serial chain with roll-pitch-yaw RRR actuated joints. Note that such an n-link robot has 3n DoF. The solver implementation is found to be reliable and accurate, consistently succesful at arriving at reasonable local minima solutions that comply with the hard constraints imposed by the joint limits and obstacles.

## Part 1
Before selecting Euler angles such that the tip of the last link reaches the specified target, the forward kinematics was symbolically calculated in order to symbolically create a cost function and a function to extract the joint positions. The cost function calculates the sum of squared differences between the target and end effector (x,y,z) positions and quaternions. The joint positions are the (x,y,z) coordinate of each joint of the snake robot which is used for object collision avoidance and visualization of the optimizer. 

## Part 2
Because the forward kinematics was calculated symbolically, the gradient for the cost function was easily obtained using the symbolic function gradient. The hessian could also be obtained symbolically, but matlab took too long to save the hessian into a function, so the hessian symbolic caluclation was commented out. 
## Part 3

## Part 4
