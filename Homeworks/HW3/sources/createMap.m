function createMap()
%CREATEMAP Create a map for the robot to navigate
%   This function creates a 2D polygonal environments map for
%   the robot motion planning problem. For different constraints
%   the map is changed accordingly.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map;

map = struct('limits', [], 'obstacles', [], 'outside', []);

map_limits = [0 100 0 100];

% Environment 1
obstacle1 = polyshape([0 0 20 20], [0 20 20 0]);
obstacle2 = polyshape([80 80 100 100], [80 100 100 80]);
obstacle3 = polyshape([0 0 20 20], [80 100 100 80]);
obstacle4 = polyshape([80 80 100 100], [0 20 20 0]);
obstacle5 = polyshape([40 40 60 60], [40 60 60 40]);

map.obstacles{1} = obstacle1;
map.obstacles{2} = obstacle2;
map.obstacles{3} = obstacle3;
map.obstacles{4} = obstacle4;
map.obstacles{5} = obstacle5;   

map.limits = map_limits;

map.outside = polyshape([0 0 100 100  NaN -200 -200 300 300], [0 100 100 0 NaN -200 300 300 -200]);


% Environment 2
% Add code here to define obstacle for environment 2

% Environment 3
% Add code here to define obstacle for environment 3

% Environment 4
% Add code here to define obstacle for environment 4

% Environment 5
% Add code here to define obstacle for environment 5

end



