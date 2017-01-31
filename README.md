# Assignment 1: Inverse Kinematics Using Optimizationasic trigonometry.
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
In order to select Euler angles such that the tip of the last link reaches the specified target (that is, solve the IK problem), the forward kinematics were symbolically developed. Representing the forward kinematics equations symbolically enables them to be automatically updated for various robot link geometries and analytically differentiable. MATLAB's "fmincon" is used to minimize a cost function encoding the normalized distance between the target pose and that of the robot at a given set of joint values. This optimization occurs under a number of hard constraints, namely joint limits and collision with spherical obstacles in the workspace. The joint limits are specified as explicit hard range constraints passed to fmincon. The spherical obstacle collision constraints are specified using fmincon's nonlinear inequality constraint facility, where the distance from each link to the nearest sphere is computed using the forward kinematics and 3D trigonometry.

For use by the IK solver routine, the symbolic FK equations and corresponding cost function are also converted to MATLAB functions. At present, the cost function computes the sum of the normalized translational and rotational pose errors. Translational distances are normalized to the diameter of the reachable workspace (the sum of all the link lengths). Rotational distances are normalized as 1-(dot(q1,q2))^2, giving a smooth metric of distance in quaternion space. Errors in normalized translational and rotational distances are weighted equally in the current implementation, but it appears that increasing the weight of the rotational distance error may improve convergence rate.

Using the default fmincon settings, our implementation was able to achieve fault-free convergence with minimal final pose error in under 10 seconds without any obstacles, and within around 30 seconds for large obstacle quantities (depending on the

## Part 2
Because the forward kinematics were calculated symbolically, the gradient for the cost function was easily obtained using the symbolic function gradient. The hessian could also be obtained symbolically, but matlab took too long to save the hessian into a function, so the hessian symbolic caluclation was commented out.

## Part 3

## Part 4
Local minima can be found for various initial guesses of link configurations. In order to try to find different local minima, n random initial guesses were generated for roll, pitch, and yaw in each link. fmincon optimization was then run for each set of initial guesses. The roll, pitch, yaw matrix of the end effector was ranked based on minimum cost. The roll, pitch, and yaw of the end effector in conditions representing the lowest cost were all different. However, visual inspection of the 3D configuration appears to be the same for the cost minima. Therefore, the optimizer was able to find the absolute cost minima given a few different initial configurations.
