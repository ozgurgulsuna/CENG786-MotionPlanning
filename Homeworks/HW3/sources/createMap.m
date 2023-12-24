function createMap()
%CREATEMAP Create a map for the robot to navigate
%   This function creates a 2D polygonal environments map for
%   the robot motion planning problem. For different constraints
%   the map is changed accordingly.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map;

map = [];
map_limits = [0 100 0 100];

% Environment 1



% Environment 2



% Environment 3



% Environment 4



% Environment 5

% Define the vertices of the polygons representing the obstacles








% % Define the vertices of the polygons representing the obstacles
% obstacle1 = [1, 1; 1, 3; 3, 3; 3, 1];
% obstacle2 = [4, 4; 4, 6; 6, 6; 6, 4];
% obstacle3 = [7, 2; 7, 4; 9, 4; 9, 2];

% % Combine the obstacle vertices into a cell array
% obstacles = {obstacle1, obstacle2, obstacle3};

% % Plot the obstacles
% figure;
% hold on;
% for i = 1:numel(obstacles)
%     obstacle = obstacles{i};
%     patch(obstacle(:, 1), obstacle(:, 2), 'r');
% end
% axis equal;
