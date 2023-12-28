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

% % Environment 1
% obstacle1 = polyshape([0 0 25 25], [0 25 25 0]);
% obstacle2 = polyshape([75 75 100 100], [75 100 100 75]);
% obstacle3 = polyshape([0 0 25 25], [75 100 100 75]);
% obstacle4 = polyshape([75 75 100 100], [0 25 25 0]);
% obstacle5 = polyshape([35 35 65 65], [35 65 65 35]);

% map.obstacles{1} = obstacle1;
% map.obstacles{2} = obstacle2;
% map.obstacles{3} = obstacle3;
% map.obstacles{4} = obstacle4;
% map.obstacles{5} = obstacle5;   

% map.limits = map_limits;

% map.outside = polyshape([0 0 100 100  NaN -200 -200 300 300], [0 100 100 0 NaN -200 300 300 -200]);


% Environment 2
obstacle1 = polyshape([15 20 20 15], [15 15 20 20]);
obstacle2 = polyshape([15 20 20 15], [80 80 85 85]);
obstacle3 = polyshape([80 85 85 80], [15 15 20 20]);
obstacle4 = polyshape([80 85 85 80], [80 80 85 85]);

obstacle5 = polyshape([15 20 20 15], [35 35 40 40]);
obstacle6 = polyshape([15 20 20 15], [60 60 65 65]);
obstacle7 = polyshape([80 85 85 80], [35 35 40 40]);
obstacle8 = polyshape([80 85 85 80], [60 60 65 65]);

obstacle9 = polyshape([35 40 40 35], [15 15 20 20]);
obstacle10 = polyshape([35 40 40 35], [80 80 85 85]);
obstacle11 = polyshape([60 65 65 60], [15 15 20 20]);
obstacle12 = polyshape([60 65 65 60], [80 80 85 85]);

obstacle13 = polyshape([35 40 40 35], [35 35 40 40]);
obstacle14 = polyshape([35 40 40 35], [60 60 65 65]);
obstacle15 = polyshape([60 65 65 60], [35 35 40 40]);
obstacle16 = polyshape([60 65 65 60], [60 60 65 65]);



map.obstacles{1} = obstacle1;
map.obstacles{2} = obstacle2;
map.obstacles{3} = obstacle3;
map.obstacles{4} = obstacle4;
map.obstacles{5} = obstacle5;
map.obstacles{6} = obstacle6;
map.obstacles{7} = obstacle7;
map.obstacles{8} = obstacle8;
map.obstacles{9} = obstacle9;
map.obstacles{10} = obstacle10;
map.obstacles{11} = obstacle11;
map.obstacles{12} = obstacle12;
map.obstacles{13} = obstacle13;
map.obstacles{14} = obstacle14;
map.obstacles{15} = obstacle15;
map.obstacles{16} = obstacle16;

map.limits = map_limits;

map.outside = polyshape([0 0 100 100  NaN -200 -200 300 300], [0 100 100 0 NaN -200 300 300 -200]);

% Environment 3
% Add code here to define obstacle for environment 3

% Environment 4
% Add code here to define obstacle for environment 4

% Environment 5
% Add code here to define obstacle for environment 5

end



