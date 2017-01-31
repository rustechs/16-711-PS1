function [r, p, y] = part1( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles )
%% Function that uses optimization to do inverse kinematics for a snake robot

%% Outputs 
  % [r, p, y] = roll, pitch, yaw vectors of the N joint angles
  %            (N link coordinate frames)
%% Inputs:
    % target: [x, y, z, q0, q1, q2, q3]' position and orientation of the end
    %    effector
    % link_length : Nx1 vectors of the lengths of the links
    % min_xxx, max_xxx are the vectors of the 
    %    limits on the roll, pitch, yaw of each link.
    % limits for a joint could be something like [-pi, pi]
    % obstacles: A Mx4 matrix where each row is [ x y z radius ] of a sphere
    %    obstacle. M obstacles.
    
    % Set initial search configuration
    % A good heuristic might be...
    q0 = ones(3*length(link_length),1);
    
    % Hard joint limit constraints
    lb = [min_roll; min_pitch; min_yaw];
    ub = [max_roll; max_pitch; max_yaw];
    
    % Solve for optimal IK solution
    % Don't use Jacobian or Hessian
    % options = optimoptions(@fmincon,'OutputFcn',@outfun,'Display','iter','MaxFunEvals',1000000,'DiffMaxChange',Inf);
    qOpt = fmincon(@(q,target)IKcost(q,target),q0,[],[]); % [],[],lb,ub,[],options);
 
    r = qOpt(1:3:end);
    p = qOpt(2:3:end);
    y = qOpt(3:3:end);
    
    
    function stop = outfun(qCurr,~,state)
        stop = false;
        
        switch state
            case 'init'
                disp('Starting optimization process');
                
                % Create figure
                figure    

            case 'iter'

                % Update figure
                updateSnakeBotDrawing(gca,jointPoses(qCurr),link_length,target,obstacles);
                
                case 'done'
               
            otherwise
                disp('wtf?');
        end
    end

end