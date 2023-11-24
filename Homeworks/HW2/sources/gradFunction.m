function [gradPot] = gradFunction(tspan, qstart, qgoal)
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
                                          
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023 


%----------------------------- BEGIN CODE ------------------------------

% Extern global variables
global sensor_range infinity arena_map obst_approx;
% global qgoal;


% Initial declarations
dimension = length(qgoal);     % Dimension of the configuration space
gradPot = zeros(dimension,1);     % Path is initialized with only 1 step


% Error checks
if length(qstart) < 2
    error('Robot configuration and goal positions must have at least 2 dimensions');
end

if length(qstart) ~= length(qgoal)
    error('Robot configuration and goal positions must have the same dimension');
end

for i = 1: length(arena_map)
    if size(arena_map{i},2) ~= length(qstart)
        error('Obstacles and robot configuration must have the same dimension');
    end
end

if length(qstart) ~= 2 && obst_approx == "EXACT"
    error('Exact obstacle representation is only available for 2D configuration spaces');
end

% Attractive Potential Gradient
gradPot = (-1*attrGrad(qstart, qgoal'));



% Repulsive Potential Gradient
for i = 1: length(arena_map)
    if obst_approx == "EXACT"
        gradPot = gradPot - 1*replGrad(qstart', i);
    elseif obst_approx == "APPROX"
        fprintf("Approximate obstacle representation is not implemented yet");
    end
end


% if abs(norm(gradPot)) < 0.1
%     gradPot = zeros(dimension,1);
% end

if norm(gradPot) > 100
    gradPot = 100*gradPot/norm(gradPot);
end

gradPot





end

%----------------------------- END OF CODE ------------------------------
