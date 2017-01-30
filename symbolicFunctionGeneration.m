function symbolicFunctionGeneration(link_length)

    %% Symbolic Forward Kinematics and Cost Function
    n = length(link_length);

    rpy = sym('rpy',[n 3],'real');
    tgt = sym('tgt',[7 1],'real');
    q = sym('q',[n 4],'real');
    p = sym('p',[n 3],'real');
    ll = sym('ll',[n 1],'real');

    q_ef = [1 0 0 0];
    p_ef = [0 0 0];

    for i = 1:n

        q(i,1) = -sin(rpy(i,1)/2)*sin(rpy(i,2)/2)*sin(rpy(i,3)/2) + cos(rpy(i,1)/2)*cos(rpy(i,2)/2)*cos(rpy(i,3)/2);
        q(i,2) = sin(rpy(i,1)/2)*cos(rpy(i,2)/2)*cos(rpy(i,3)/2) + sin(rpy(i,2)/2)*sin(rpy(i,3)/2)*cos(rpy(i,1)/2);
        q(i,3) = -sin(rpy(i,1)/2)*sin(rpy(i,3)/2)*cos(rpy(i,2)/2) + sin(rpy(i,2)/2)*cos(rpy(i,1)/2)*cos(rpy(i,3)/2);
        q(i,4) = sin(rpy(i,1)/2)*sin(rpy(i,2)/2)*cos(rpy(i,3)/2) + sin(rpy(i,3)/2)*cos(rpy(i,1)/2)*cos(rpy(i,2)/2);


        % Might need to flip order
        q_ef = ([q(i,1) -q(i,2) -q(i,3) -q(i,4);...
                 q(i,2)  q(i,1) -q(i,4)  q(i,3);...  
                 q(i,3)  q(i,4)  q(i,1) -q(i,2);...
                 q(i,4) -q(i,3)  q(i,2)  q(i,1)]*q_ef')';

    %    q_ef(1) = q_ef(1)*q(i,1)-q_ef(2)*q(i,2)-q_ef(3)*q(i,3)-q_ef(4)*q(i,4);
    %    q_ef(2) = q_ef(1)*q(i,2)+q_ef(2)*q(i,1)-q_ef(3)*q(i,4)+q_ef(4)*q(i,3);
    %    q_ef(3) = q_ef(1)*q(i,3)+q_ef(2)*q(i,4)+q_ef(3)*q(i,1)-q_ef(4)*q(i,2);
    %    q_ef(4) = q_ef(1)*q(i,4)-q_ef(2)*q(i,3)+q_ef(3)*q(i,2)+q_ef(4)*q(i,1);

        % Rotate link displacement vector
        p(i,:) = ([(1-2*q(i,3)^2-2*q(i,4)^2)        2*(q(i,2)*q(i,3)+q(i,1)*q(i,4)) 2*(q(i,2)*q(i,4)-q(i,1)*q(i,3)); ...
                   2*(q(i,2)*q(i,3)-q(i,1)*q(i,4))  (1-2*q(i,2)^2-2*q(i,4)^2)       2*(q(i,3)*q(i,4)+q(i,1)*q(i,2)); ...
                   2*(q(i,2)*q(i,4)+q(i,1)*q(i,3))  2*(q(i,3)*q(i,4)-q(i,1)*q(i,2)) (1-2*q(i,2)^2-2*q(i,3)^2)]*[link_length(i) 0 0]')';

        p_ef = p_ef + p(i,:);

    end

    rpy = reshape(rpy',3*n,1);
    
    jointPoses = matlabFunction(p,'File','jointPoses','Vars',{rpy});

    FK = symfun([p_ef q_ef]',rpy);
    C = symfun(sum((tgt-[p_ef q_ef]').^2),[tgt; rpy]);
    Cgrad = symfun(gradient(sum((tgt-[p_ef q_ef]').^2),rpy),[tgt; rpy]);
    % Chess = symfun(hessian(sum((tgt-[p_ef q_ef]').^2),rpy),[tgt; rpy]);

    FK_fn = matlabFunction(FK,'File','FK_fn','Vars',{rpy});
    C_fn = matlabFunction(C,'File','C_fn','Vars',{tgt,rpy});
    Cgrad_fn = matlabFunction(Cgrad,'File','Cgrad_fn','Vars',{tgt,rpy});
    % Chess_fn = matlabFunction(Chess,'File','Chess_fn','Vars',{tgt,rpy});

end