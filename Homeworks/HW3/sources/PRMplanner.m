% Date: 2023-12
% Description: Probabilistic roadmap planner for multiple link robots in a 2D plane
%   _____  _______ _______ _______ _______ _______  
%  |_____| |______ |______ |______ |______    |     
%  |     | ______| ______| |______ ______|    |    
%
% This script implements a simple PRM (Probabilistic Roadmap) planner for a planar two-link robot
% that can freely move in a polygonal environment. The robot consists of two rigid links connected
% with a rotational joint.
%
% The PRM planner generates a roadmap of valid configurations in the configuration space and uses
% this roadmap to find a feasible path from a start configuration to a goal configuration. The planner
% uses random sampling to generate configurations and collision checking to ensure that the generated
% configurations are valid and collision-free.
%
% Note: This script requires the following helper functions:
%       - generateRandomConfigurations
%       - checkCollision
%       - findNearestNeighbor
%       - constructPath
%       - plotEnvironment
%       - plotRobot
%       - plotPath
%  
% Author: Ozgur Gulsuna
% GitHub: ozgurgulsuna 
% This implementation is part of the CENG786 course homework assignment.

clc;
clear;

% global variables
global map; % map of the environment
global robot_params; % parameters of the robots
global q_init; % initial configuration
global q_goal; % goal configuration

% Create map
createMap();
drawMap();

% Create robot
% createRobot();

% initial and goal configurations
q_init = [0.1, 0.1];
q_goal = [0.9, 0.9];




