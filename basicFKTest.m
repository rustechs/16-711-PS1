%% Basic FK Testing

clc; clear; close all

% Test out some FK calculations

% Robot definition
link_length = [2 1];

target = [5 5 5 1 0 0 0];
obstacles = [];

% Call this to generate the symbolic FK and cost functions, as well as
% the corresponding matlab function files.
% NOTE: NEED TO RERUN IF LINK_LENGTH CHANGES

% symbolicFunctionGeneration(link_length);

rpyTestSet = [0 0 0 0 0 0; ...
              0 pi/2 0 0 0 0; ...
              0 0 pi/2 0 0 0]';

figure

for rpy = rpyTestSet
    
    disp('Current RPY values:');
    disp(rpy);
    
    % Update figure
    updateSnakeBotDrawing(gca,jointPoses(rpy),link_length,target,obstacles);
    disp('Press Anything to Continue...');
    pause;
    
end