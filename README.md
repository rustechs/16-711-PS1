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

## Overview
Traditional inverse kinematics (IK) solvers for serial link mechanisms are implemented using lookup tables or explicit analytically-derived functions. Advances in symbolic equation solvers and optimization algorithms enable an alternative, more automated and less cognitively involved approach. In this assignment, a 3D n-link serial chain IK solver (and object collision avoidance) is treated as a constrained optimization problem, implemented using MATLAB's "fmincon" function and the "Symbolic Toolbox" to automatically derive the forward kinematics (FK) for an arbitrary n-link serial chain with roll-pitch-yaw RRR actuated joints.

## Part 1

## Part 2

## Part 3

## Part 4
