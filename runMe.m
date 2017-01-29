%% runMe.m
%
%  This runs everything for this problem set, and creates beautiful
%  graphics.


%% Part 1

% Generate Data

% Plot Stuff

% Create figure
h = figure;

% Create axes
ax = axes;

grid on

hold on

% Size axes
% Should depend on max arm length

% Draw end-effector target

% Draw robot origin

% Draw obstacles


% Update figure
for i = 1:size(r,1)
    updateSnakeBotDrawing(ax);
end