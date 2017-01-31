%% runMe.m
%
%  This runs everything for this problem set

%% Clean up
clc; clear; close all

%% Sample Inputs
obstNum = 4;
maxObstRad = 0.5;

%obstacles = [1 1 1 0.25; 6 6 6 0.5];
link_length = [3 2 1];

range = sum(link_length);
obstacles = [normrnd(0,range/3,[obstNum 3]) normrnd(0.25,0.1,[obstNum 1])];
target = [0.75*range*rand(1,3) 0 0.5 0.5 0]';

%target = [2 1 2 0 -1 0 0]';
%obstacles = [-R+2*R*rand([obstNum,3]) maxObstRad*rand([obstNum,1])];

min_roll = -pi*ones(length(link_length),1);
max_roll = pi*ones(length(link_length),1);
min_pitch = -pi*ones(length(link_length),1);
max_pitch = pi*ones(length(link_length),1);
min_yaw = -pi*ones(length(link_length),1);
max_yaw = pi*ones(length(link_length),1);

% Call this to generate the symbolic FK and cost functions, as well as
% the corresponding matlab function files

% symbolicFunctionGeneration(link_length);

%% Part 1

disp('Starting Part 1');

tic

[r, p, y] = part1( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);

toc

disp('Optimal joint (RPY) positions found:');
disp('Roll');
disp(r);
disp('Pitch');
disp(p);
disp('Yaw');
disp(y);

%% Part 2

disp('Starting Part 2');

tic

[r, p, y] = part2( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);

toc

disp('Optimal joint (RPY) positions found:');
disp('Roll');
disp(r);
disp('Pitch');
disp(p);
disp('Yaw');
disp(y);

%% Part 3

disp('Starting Part 3');

tic

[r, p, y] = part3( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);

toc

disp('Optimal joint (RPY) positions found:');
disp('Roll');
disp(r);
disp('Pitch');
disp(p);
disp('Yaw');
disp(y);

%% Part 4

disp('Starting Part 4');

tic

[r, p, y] = part4( target, link_length, min_roll, max_roll, min_pitch, max_pitch, min_yaw, max_yaw, obstacles);

toc

disp('Optimal joint (RPY) positions found:');
disp('Roll');
disp(r);
disp('Pitch');
disp(p);
disp('Yaw');
disp(y);