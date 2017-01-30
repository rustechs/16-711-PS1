function [cost,J,H] = IKcost(q,target)
% Defines cost function for IK optimizer

    pose = double(FKquat(q));
    
    % Basic cost is sum of squares of end-effector pose error from target
    cost = sum((target-pose).^2);
    
    % Analytical Jacobian and Hessian
    % For part 2 and beyond
    J = [];
    H = [];

end