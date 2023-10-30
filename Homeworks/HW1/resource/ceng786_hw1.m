
%  This file is an example template you can use for your solution.  It
% provides wrapper functions as well as various utilities that may be
% useful for your implementation
%
%  Your solution can implement the following function
%
%   [x,y] = bug_planner( qstart, qgoal )
%
% This function is expected to return a collision free path
% starting from qstart and ending in qgoal, both supplied as row
% vectors. The returned values should be row vectors with the x and
% y coordinates of the robot along this path, as it would have
% followed with your planner in effect.
%
% In implementing these functions, you are only allowed to use the
% following functions supplied with the homework (as well as any
% others you may choose to implement yourself):
%
%  draw_arena, draw_range_map, read_sensor
%
%  Furthermore, you are only allowed to access the following global
% variables:
%
%  sensor_range, infinity
%
%  You may of course write your own functions as you see fit as long
% as you do not cheat by accessing the global arena map and follow
% submission guidelines for this homework. You may also use various
% builtin Matlab functions as you see fit.
%
%  Please follow good coding standards and document your code with
% useful and clear comments in English.
%
% All quantities are in MKS units unless otherwise specified.
%  distances    : m
%  angles       : rad
%  speed        : m/s
%  acceleration : m/s^2
%

function ceng786_hw1;
clear
clc
% Global Parameters Declarations -----------------
global sensor_range;  % Determines limited sensor range
global arena_limits;  % Boundaries of the arena: [xmin xmax ymin ymax]
global arena_map;     % Description of obstacles in the environment
global infinity;      % Large value to be used as 'infinity'
global qstart qgoal;  % Start and goal configurations

% Parameter values to be used for the homework ---
sensor_range = 0.25;
infinity = 1e5;
arena_limits = [0 10 0 10];
qstart = [0.5 0.5];
qgoal  = [8 9];
n = 10; % time step for animation

% Invoking your solutions for the example arena ------------------------
% init_arena();
tic
[x_m1_b1, y_m1_b1] = bug_one_fast( qstart, qgoal );
toc
figure(1);
clf;
draw_arena;
xticks([0 1 2 3 4 5 6 7 8 9 10])
yticks([0 1 2 3 4 5 6 7 8 9 10])
zticks([0 1 2 3 4 5 6 7 8 9 10])
xlim([0 10])
ylim([0 10])
zlim([0 10])
hold on; plot( x_m1_b1, y_m1_b1 );

figure(2);
clf;

for j=1:size(arena_map,2)
    arena_map{j}(:,1) = arena_map{j}(:,1) - 0.02*length(x_m1_b1)*0.1;
    arena_map{j}(:,2) = arena_map{j}(:,2) - 0.02*length(x_m1_b1)*0.0018;
end
for i = 1:length(x_m1_b1)/n
  draw_range_map( [x_m1_b1(n*i) y_m1_b1(n*i)], 30 );
  for j=1:size(arena_map,2)
    arena_map{j}(:,1) = arena_map{j}(:,1) + 0.02*n*0.1;
    arena_map{j}(:,2) = arena_map{j}(:,2) + 0.02*n*0.0018;
end
  drawnow;
end

end
%% -----------------------------------------------------------------
% init_arena
%
% Definition of the example arena map for Homework 1
%
%% -----------------------------------------------------------------
function init_arena;
global arena_map qstart qgoal;

arena_map = [];
% arena_map{1} = ...
% [ 2.0392  3.5234; 1.8318  5.0731; 2.0161  6.7982; 2.6152  8.1433; ...
%   3.5369  8.9035; 5.0576  9.0205; 6.3249  8.8158; 7.4078  7.8509; ...
%   8.0300  6.8275; 8.1452  4.8977; 8.0760  3.4357; 7.6613  2.1784; ...
%   6.8548  1.1842; 5.3571  0.8041; 4.3433  1.0965; 3.6751  2.5000; ...
%   3.5369  3.7281; 3.4447  4.8099; 3.9977  6.2135; 4.5968  6.7982; ...
%   5.0115  6.5936; 4.5968  5.6287; 4.1820  4.6930; 4.0668  3.7865; ...
%   4.1129  2.5877; 4.5276  1.7398; 5.4954  1.3596; 6.6705  1.6228; ...
%   7.2465  2.4415; 7.6843  3.6696; 7.7765  5.1608; 7.6843  6.3304; ...
%   6.9700  7.3246; 6.1866  8.1140; 5.0346  8.3480; 3.7673  8.2018; ...
%   3.1682  7.7924; 2.4539  6.7105; 2.3848  5.1023];
% arena_map{2} = ...
% [ 5.2889  5.1131; 4.7111  4.2839; 4.8869  3.5302; ...
%   6.1683  3.9070; 6.1432  5.0377 ];
% arena_map{3} = ...
% [ 5.1382    7.2487; 5.3392  6.8719; 5.3392  6.3693; ...
%     5.2889  6.0427; 5.5402  6.0930; 5.7412  6.4447; ...
%     5.7412  6.9724; 5.4899  7.2990; 5.2638  7.4749 ];

% arena_map{1} =  ...
%    [3      8;   ...
%     3      1;   ...
%     3.5    1;   ...
%     3.5    8 ];
% 
% arena_map{2} =   ...
%    [5      4;    ...
%     4.5    4;    ...
%     4.5    2;    ...
%     6.5    2;    ...
%     6.5    2.5;  ...
%     5    2.5 ];
% arena_map{3} = ...
%     [1     2; ...
%      1     5; ...
%      1     6; ...
%      1     7; ...
%      1     9; ...
%      2     3; ...
%      3     4; ...
%      3     6; ...
%      3     7; ...
%      3    10; ...
%      4     1; ...
%      4     5; ...
%      4     6; ...
%      4     7; ...
%      4     9; ...
%      5     1; ...
%      5     4; ...
%      5     6; ...
%      6     1; ...
%      6     3; ...
%      6     4; ...
%      6     7; ...
%      7     1; ...
%      7     3; ...
%      7     4; ...
%      7     8; ...
%      8     3; ...
%      8     9; ...
%      9     1; ...
%      9     4; ...
%      9    10; ...
%     10     3; ...
%     10     5];


% arena_map{1} = [ ...
%             4.7006    4.9363 ; ...
%             4.8408    4.6561 ; ...
%             5.5287    4.5159 ; ...
%             5.5159    5.5605 ; ...
%             4.6497    5.2930 ; ...
%             4.6879    4.5032 ; ...
%             3.9745    4.5159 ; ...
%             3.8599    5.7389 ; ...
%             5.6815    6.4395 ; ...
%             7.9108    4.8089 ; ...
%             7.3758    3.3949 ; ...
%             4.9299    3.2930 ; ...
%             4.0510    4.3121 ; ...
%             4.0255    5.2166 ];



% stargazer
arena_map{1} = [ ...
    6.6879    6.8726 ; ...
    7.9363    7.3439 ; ...
    6.9554    8.6561 ; ...
    6.5860    9.3057 ; ...
    5.8471    8.1975 ; ...
    6.5987    7.6497 ; ...
    6.2930    7.1401 ; ...
    5.8726    6.8089 ; ...
    6.7771    6.8217 ];

arena_map{2} = [ ...
    2.8662    4.0064 ; ...
    4.0382    4.4268 ; ...
    4.3439    4.8599 ; ...
    4.5350    7.0255 ; ...
    4.4968    7.3949 ; ...
    3.7070    8.1847 ; ...
    3.0828    8.0191 ; ...
    2.7261    7.2293 ; ...
    2.4331    6.3885 ; ...
    2.1911    5.4968 ; ...
    2.7006    4.5669 ];

arena_map{3} = [ ...
    5.9490    1.3439 ; ...
    5.6178    2.7580 ; ...
    5.6815    3.6624 ; ...
    6.6879    4.7707 ; ...
    6.3567    5.7006 ; ...
    7.3121    5.0000 ; ...
    7.7325    4.0064 ; ...
    8.6115    3.1146 ; ...
    7.8726    2.2866 ; ...
    6.1274    1.3312];

arena_map{4} = [ ...
    1.5414    1.6752 ; ...
    1.6306    2.2866 ; ...
    1.9873    2.6943 ; ...
    2.9936    2.4650 ; ...
    3.6306    1.9427 ; ...
    3.0191    1.5096 ; ...
    2.4204    1.5987 ; ...
    2.4841    0.7962];


qstart = [0.5 0.5];
qgoal  = [8 9];

end
