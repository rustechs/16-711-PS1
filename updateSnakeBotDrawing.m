function [] = updateSnakeBotDrawing(ax,r,p,y,link_length)
%% updateSnakeBotDrawing.m
%
%  Updates 3D drawing of snake robot in figure h.

    % How many links
    n = length(link_length);

    % Compute the link endpoint locations
    % Converts from RPY and link_lengths to cartesian coordinates
    % [xpi ypi zpi xdi ydi zdi]
    links = [];

    % might need to transpose each input
    plot3(ax,links(:,[1 4]),links(:,[2,5]),links(:,[3,6]));

    drawnow limitrate
   