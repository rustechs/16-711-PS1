%% runMe.m
%
%  This runs everything for this problem set

%% Clean up
clc; clear; close all

%% Sample Inputs
target = [1 1 1 1 0 0 0];
obstacles = [1 1 1 0.25; 6 6 6 0.5];
link_length = [3 2 1];

min_roll = [-pi/2 -pi/2 -pi/2];
max_roll = [ pi/2  pi/2 pi/2];
min_pitch = [-pi/2 -pi/2 -pi/2];
max_pitch = [ pi/2  pi/2 pi/2];
min_yaw = [-pi/2 -pi/2 -pi/2];
max_yaw = [ pi/2  pi/2 pi/2];

% Call this to generate the symbolic FK and cost functions, as well as
% the corresponding matlab function files

% symbolicFunctionGeneration(link_length);

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

