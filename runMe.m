%% runMe.m
%
%  This runs everything for this problem set

%% Clean up
clc; clear; close all

%% Sample Inputs
target = [2 1 2 0 -1 0 0]';
obstacles = [1 1 1 0.25; 6 6 6 0.5];
link_length = [3 2 1];

min_roll = -pi*ones(length(link_length),1);
max_roll = pi*ones(length(link_length),1);
min_pitch = -pi*ones(length(link_length),1);
max_pitch = pi*ones(length(link_length),1);
min_yaw = -pi*ones(length(link_length),1);
max_yaw = pi*ones(length(link_length),1);

% Call this to generate the symbolic FK and cost functions, as well as
% the corresponding matlab function files

symbolicFunctionGeneration(link_length);

%% Part 1

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

