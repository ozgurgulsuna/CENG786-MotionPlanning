function [potFunc] = potFunction(tspan, qstart, qgoal)
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
%   - Output: potFunc  = [P_1 P_2 ... P_n]  gradient of the potential function
%                                            
                                          
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023 


%----------------------------- BEGIN CODE ------------------------------

% Extern global variables
global sensor_range infinity arena_map obst_approx;
% global qgoal;


% Attractive Potential Gradient
potFunc = attrPot(qstart, qgoal');



% Repulsive Potential Gradient
for i = 1: length(arena_map)
    potFunc = potFunc + 1*replPot(qstart', i);
end


if norm(potFunc) > 100
    potFunc = 100;
elseif norm(potFunc) < 0.1
    potFunc = 0;
end

% for m = 1: length(arena_map)
%     if potFunc ~= 100
%         if inpolygon(qstart(1), qstart(2), arena_map{m}(:,1), arena_map{m}(:,2))
%             potFunc = 100;
%         end
%     end
% end

% if obst_approx == "APPROX"
%     for m = 1: length(arena_map)
%         if potFunc ~= 100
%             if inpolygon(qstart(1), qstart(2), arena_map{m}(:,1), arena_map{m}(:,2))
%                 potFunc = 100;
%             end
%         end
%     end
% end




end

%----------------------------- END OF CODE ------------------------------
