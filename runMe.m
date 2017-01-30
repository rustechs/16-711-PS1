%% runMe.m
%
%  This runs everything for this problem set, and creates beautiful
%  graphics.

%% Clean up
clc; clear; close all

%% Sample Inputs
target = [5 5 5];
obstacles = [1 1 1 0.25; 6 6 6 0.5];
link_length = [4 3 2 1];

%% Symbolic Forward Kinematics
n = length(link_length);

rpy = sym('rpy',[n 3]);
q = sym('q',[n 4]);
p = sym('p',[n 3]);
ll = sym('ll',[n 1]);

q_ef = [1 0 0 0];
p_ef = [0 0 0];

for i = 1:n
   
   p(i,:) = 
    
   p_ef = p_ef + p(i,:);
    
   q(i,1) = -sin(rpy(i,1)/2)*sin(rpy(i,2)/2)*sin(rpy(i,3)/2) + cos(rpy(i,1)/2)*cos(rpy(i,2)/2)*cos(rpy(i,3)/2);
   q(i,2) = sin(rpy(i,1)/2)*cos(rpy(i,2)/2)*cos(rpy(i,3)/2) + sin(rpy(i,2)/2)*sin(rpy(i,3)/2)*cos(rpy(i,1)/2);
   q(i,3) = -sin(rpy(i,1)/2)*sin(rpy(i,3)/2)*cos(rpy(i,2)/2) + sin(rpy(i,2)/2)*cos(rpy(i,1)/2)*cos(rpy(i,3)/2);
   q(i,4) = sin(rpy(i,1)/2)*sin(rpy(i,2)/2)*cos(rpy(i,3)/2) + sin(rpy(i,3)/2)*cos(rpy(i,1)/2)*cos(rpy(i,2)/2);
   
   % Write this out yourself
   q_ef(1) = q_ef(1)*q(i,1)-q_ef(2)*q(i,2)-q_ef(3)*q(i,3)-q_ef(4)*q(i,4);
   q_ef(2) = q_ef(1)*q(i,2)+q_ef(2)*q(i,1)-q_ef(3)*q(i,4)+q_ef(4)*q(i,3);
   q_ef(3) = q_ef(1)*q(i,3)+q_ef(2)*q(i,4)+q_ef(3)*q(i,1)-q_ef(4)*q(i,2);
   q_ef(4) = q_ef(1)*q(i,4)-q_ef(2)*q(i,3)+q_ef(3)*q(i,2)+q_ef(4)*q(i,1);
end

sym FK(q)
FK(q) = [p_ef q_ef]';

FK = matlabFunction(FK(q));



%% Part 1

tic

[r, p, y] = part1( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);

toc

% Plot Stuff

% Create figure
h = figure;

% Create axes
ax = axes;

grid on
hold on

% Size axes
R = sum(link_length);
axis(R*[-1 1 -1 1 -1 1]);

% Draw end-effector target
plot3(target(1),target(2),target(3),'MarkerSize',3);

% Draw robot origin
plot3(0,0,0,'s','MarkerSize',10);

% Draw obstacles
for o = obstacles'
    [sX,sY,sZ] = sphere;
    surf(ax,o(4)*sX+o(1),o(4)*sY+o(2),o(4)*sZ+o(3));
end

% Update figure
for i = 1:size(r,1)
    updateSnakeBotDrawing(ax,r,p,y,link_length);
end


%% Part 2

