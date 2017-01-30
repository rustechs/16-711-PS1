function cost = IKcost(q,target)
% Defines cost function for IK optimizer
    
    % Basic cost is sum of squares of end-effector pose error from target
    cost = C_fn(target,q);
    
    % Analytical Jacobian and Hessian
    % For part 2 and beyond
    J = Cgrad_fn(target,q);
%   H = Chess_fn(target,q);

end