function createMap()
%CREATEMAP Create a map for the robot to navigate
%   This function creates a 2D polygonal environments map for
%   the robot motion planning problem. For different constraints
%   the map is changed accordingly.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map;

map = struct('limits', [], 'obstacles', []);

map_limits = [0 100 0 100];

% Environment 1
obstacle1 = [0 0; 0 20; 20 20; 20 0];
obstacle2 = [80 80; 80 100; 100 100; 100 80];
obstacle3 = [0 80; 0 100; 20 100; 20 80];
obstacle4 = [80 0; 80 20; 100 20; 100 0];
obstacle5 = [40 40; 40 60; 60 60; 60 40];

map.obstacles{1} = obstacle1;
map.obstacles{2} = obstacle2;
map.obstacles{3} = obstacle3;
map.obstacles{4} = obstacle4;
map.obstacles{5} = obstacle5;   

map.limits = map_limits;


% Environment 2
% Add code here to define obstacle for environment 2

% Environment 3
% Add code here to define obstacle for environment 3

% Environment 4
% Add code here to define obstacle for environment 4

% Environment 5
% Add code here to define obstacle for environment 5

end



