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

% initial and goal configurations
p_init = [robotTopology.nodes(:,1) robotTopology.nodes(:,2) robotTopology.nodes(:,3) ] ; % p = [foot1 foot2 foot3]
s_goal = [ 50 50 50]; % goal coordinates (s = [x y z])

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
generateTerrain('sch.jpg');
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
hold off
