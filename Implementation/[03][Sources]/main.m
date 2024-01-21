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
s_goal = [38.25 78.25 0.8425]; % goal coordinates (s = [x y z])

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
generateTerrain('shackleton-round.jpg');
% load('terrainMesh.mat')

PRTplanner(p_init, s_goal);


% Simulation

% Analysis

% Plotting




% % quick plot
% figure(1)
% clf
% hold on
% axis equal
% grid 
% xlabel('x')
% ylabel('y') 
% zlabel('z')
% view(3)
% for i = 1:M
%     plot3(robotTopology.nodes(robotTopology.connectivity(i,:),1), ...
%           robotTopology.nodes(robotTopology.connectivity(i,:),2), ...
%           robotTopology.nodes(robotTopology.connectivity(i,:),3), ...
%           'k-', 'LineWidth', 2)
% end


