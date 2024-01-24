%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%|├─╯└─╯├─╯╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤└─╯├─╯╱╱╱╱├─╯├─┤└─╱╱╱╱╱╱╱╱╱└─╯├─╯╱╱╱╱├─╯├─┤└─╯├─╯╱╱╱╱╱╱╱╱├─╯├─┤└─╱╱   |
%|┴─╮┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮├─╮┴─┤┌─╱╱╱╱╱╱╱╱╱┌─╮┴─╮╭┬─╮├─╮┴─╮╭┬─╮╱╱╱╱╱╱╱╱├─╮┴─╮╭┬─╮├─╮ |
%|─╯└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
%|                                                                                                       |
%|                                      Variable Geometry Truss Robot                                    |
%|                                              Motion Planning                                          |
%|                                                                                                       |
%|───╮┌─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮├─╮┴─┤┌─╱╱╱╱╱╱╱╱╱┌─╮┴─╮╭┬─╮├─╮┴─╮╭┬─╮╱╱╱╱╱╱╱╱├─╮┴─╮╭┬─╮├─╮ ├─|
%|─╯└─╯╰┴─╯╰─────────╯╰─╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰────────╯╰──╯┴┴┴┴──╯╰─|
%|                                                                                                       |
%|                  Author: Ozgur Gulsuna                                                                |
%|                  Date: 07-01-2024                                                                     |
%|                                                                                                       |
%|                  Description:                                                                         |
%|                  This MATLAB code implements a variable truss topology robot, allowing                |
%|                  for flexibility in the design of the truss structure.                                |
%|                                                                                                       |
%|                  Usage:                                                                               |
%|                  - Ensure you have the required toolboxes and dependencies installed.                 |
%|                  - Set up your robot parameters and desired truss topology.                           |
%|                  - Run the script to simulate and analyze the robot's performance.                    |
%|                                                                                                       |
%|                  Note:                                                                                |
%|                  Customize this header with relevant information for your application.                |
%|                                                                                                       |
%|                  Acknowledgements:                                                                    |
%|                  This code was developed as part of the CENG786 - Robot Motion Planning and Control   |
%|                  course at the Middle East Technical University, Ankara, Turkey.                      |
%|                                                                                                       |
%|                                                                                                       |
%|                                                                                                       |
%|                                                                                                       |
%|╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭─╮┬─|
%|─╯└─╯├─╯╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├──╮┤└─╯├─╯╱╱╱╱├─╯├─┤└─╱╱╱╱╱╱╱╱╱└─╯├─╯╱╱╱╱├─╯├──╮┤└─╯├─╯╱╱╱╱╱╱╱╱├─╯├─┤└─╱╱|
%└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴┴┴┴──╯╰─╯

clear 
close all
clc

% global variables
global robotTopology;
global terrain;
global mesh;
global tree;

% initial and goal configurations
p_init = [[9.24587 14.0343 1.29837] ; [8.14445 17.1578 1.5472] ; [11.4071 16.5581 1.34989]]; % initial configuration (p = [x y z])
% p_init = 3*[[1 1 0.3] ; [1+sqrt(3)/2 3/2 0.3] ; [1 2 0.3]  ]; % initial configuration (p = [x y z])
% p_init = p_init + 2*[[10 10 0] ; [10 10 0] ; [10 10 0]];
s_goal = [38.25 78.25 0.8425]; % goal coordinates (s = [x y z])
robot_size = 2; % size of the robot

% figure
% hold on
% scatter3(p_init(:,1), p_init(:,2), p_init(:,3), 'filled', 'MarkerFaceColor', 'r')

% Create the VGT Robot
createRobot()

% Robot Parameters
N = size(robotTopology.nodes,1);  % Number of nodes
M = size(robotTopology.members,1); % Number of members
% L = zeros(M,1); % Member lengths
x = [robotTopology.nodes(:,1); robotTopology.nodes(:,2); robotTopology.nodes(:,3)]; % Node coordinates


% Inverse Kinematics (IK) Jacobian
J = inverseKinematics();

L = J * x;

% CoM = calculateCoM();

% Locomotion
    % Constraints
    % Optimization
    % Actuation 



% Path Planning
generateTerrain('shackcorrected.jpg');
% generateTerrain('plain_arena_2.png');
% load('terrainMesh.mat')

PRTplanner(p_init, s_goal, robot_size);


% Simulation

% Analysis

% Plotting


% % rotate the robot on its side
% yaw_angle = 0; % degrees
% pitch_angle = 45; % degrees
% roll_angle = 35.15; % degrees

% rotation_matrix = [cosd(yaw_angle) -sind(yaw_angle) 0; sind(yaw_angle) cosd(yaw_angle) 0; 0 0 1] * ...
%                   [cosd(pitch_angle) 0 sind(pitch_angle); 0 1 0; -sind(pitch_angle) 0 cosd(pitch_angle)] * ...
%                   [1 0 0; 0 cosd(roll_angle) -sind(roll_angle); 0 sind(roll_angle) cosd(roll_angle)];

% robotTopology.nodes = robotTopology.nodes * rotation_matrix;




% % quick plot
% figure(1)
% clf
% hold on
% axis equal
% grid 
% xlabel('x')
% ylabel('y') 
% zlabel('z')
% % view(-215,25.8)
% view(-142, 32)
% % campos([8.330841562736607,11.216880804505152,6.762704086483187])
% campos([-8.920211876837246,11.535634218319535,9.346601695454195 ])
% % camera view angle:
% camva(9.50594848470686)
% xlim([-1 1])
% ylim([-1 1])
% zlim([-1 1])


% for i = 1:M
%     plot3(robotTopology.nodes(robotTopology.connectivity(i,:),1), ...
%           robotTopology.nodes(robotTopology.connectivity(i,:),2), ...
%           robotTopology.nodes(robotTopology.connectivity(i,:),3), ...
%           'k-', 'LineWidth', 1.5)
% end

% exportgraphics(gcf,'D:\2023-24\Education\CENG786\Implementation\[04][Report]\images\01-blank.pdf','ContentType','vector')



