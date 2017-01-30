function [] = updateSnakeBotDrawing(ax,jointPoses,link_length,target,obstacles)
%% updateSnakeBotDrawing.m
%
%  Updates 3D drawing of snake robot in figure h.

    axes(ax);

    cla
    
    grid on
    box on
    hold on
    
    % Size axes
    R = sum(link_length);
    axis(1.1*R*[-1 1 -1 1 -1 1]);
    
    axis square
    
    % Draw end-effector target
    plot3(target(1),target(2),target(3),'p','MarkerSize',10,'MarkerFaceColor','m');

    % Draw obstacles
    for o = obstacles'
        [sX,sY,sZ] = sphere;
        surf(o(4)*sX+o(1),o(4)*sY+o(2),o(4)*sZ+o(3),'FaceColor','red','EdgeColor','none');
    end

    n = size(jointPoses,1);
    
    links = zeros(n,6);
    
    % Compute the link endpoint locations
    % [xpi ypi zpi xdi ydi zdi]
    links(1,4:6) = jointPoses(1,:);
    for i = 2:n
        links(i,:) = [jointPoses(i-1,:) jointPoses(i,:)];
    end
    
    % might need to transpose each input
    plot3(ax,links(:,[1 4])',links(:,[2,5])',links(:,[3,6])','-o','LineWidth',10,'MarkerSize',10,'MarkerFacecolor','k');

    drawnow limitrate
   