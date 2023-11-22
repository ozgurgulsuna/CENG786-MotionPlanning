function [gradPot] = potFunction(tspan, qstart, qgoal)
%ATTR_REPL Additive attraction and repulsion path planning implementation
%  This function implements the additive attraction and repulsion method
%  for path planning. The function takes as input the start and goal
%  positions of the robot and returns the path as an n dimensional vector.
%  The number of robot and the goal is assumed to be 1.
%       
%   - Input:    qstart = [s_1 s_2 ... s_n]  starting coordinates of the robot,
%               qgoal  = [g_1 g_2 ... g_n]  goal coordinates of the robot,
%                                           where "n" is the dimension of the
%                                           configuration space.
%
%   - Output: gradPot  = [P_1 P_2 ... P_n]  gradient of the potential function
%                                            
%                                            
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023 

% Extern global variables
global sensor_range ; 
global infinity ;
global arena_map ;
% global qgoal;
qgoal 

% Initial declarations
dimension = length(qgoal);     % Dimension of the configuration space
gradPot = zeros(dimension,1);     % Path is initialized with only 1 step


% Attractive Potential Gradient
for n = 1: dimension
    gradPot(n) = -1*attrGrad(qstart(n), qgoal(n));
end

% Repulsive Potential Gradient
for i = 1: length(arena_map)
    for n = 1: dimension
        gradPot(n) = gradPot(n) + replGrad(qstart(n), arena_map(i).center(n), arena_map(i).radius);
    end
end





end