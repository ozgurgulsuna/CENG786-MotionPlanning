%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%|├─╯└─╯├─╯╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤└─╯├─╯╱╱╱╱├─╯├─┤└─╱╱╱╱╱╱╱╱╱└─╯├─╯╱╱╱╱├─╯├─┤└─╯├─╯╱╱╱╱╱╱╱╱├─╯├─┤└─╱╱   |
%|┴─╮┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮├─╮┴─┤┌─╱╱╱╱╱╱╱╱╱┌─╮┴─╮╭┬─╮├─╮┴─╮╭┬─╮╱╱╱╱╱╱╱╱├─╮┴─╮╭┬─╮├─╮ |
%|─╯└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
%|                                                                                                       |
%|                                      Variable Truss Topology Robot                                    |
%|                                         MATLAB Implementation                                         |
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
%|                  This code was developed as part of the MECH 6313 - Advanced Robotics course at       |
%|                  Carleton University.                                                                 |
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

global robotTopology



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






% quick plot
figure(1)
clf
hold on
axis equal
grid on
xlabel('x')
ylabel('y') 
zlabel('z')
view(3)
for i = 1:M
    plot3(robotTopology.nodes(robotTopology.connectivity(i,:),1), ...
          robotTopology.nodes(robotTopology.connectivity(i,:),2), ...
          robotTopology.nodes(robotTopology.connectivity(i,:),3), ...
          'k-', 'LineWidth', 2)
end
hold off
