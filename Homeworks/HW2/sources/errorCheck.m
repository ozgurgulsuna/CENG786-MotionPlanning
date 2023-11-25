function errorCheck()
% errorCheck.m

global qstart qgoal arena_map obst_approx potFunc dimension;
% Initial declarations
dimension_start = length(qstart);     % Dimension of the robot configuration space
dimension_goal = length(qgoal);       % Dimension of the goal configuration space
dimension_map = size(arena_map{1},2); % Dimension of the obstacle configuration space


% Error checks

if dimension_start ~= dimension_goal
    error('Robot configuration and goal positions must have the same dimension');
end

if dimension_start ~= dimension_map
    error('Obstacles and robot configuration must have the same dimension');
end

if dimension_goal ~= dimension_map
    error('Obstacles and goal positions must have the same dimension');
end

if dimension_start < 2
    error('Robot configuration must have at least 2 dimensions');
end

if dimension_goal < 2
    error('Goal configuration must have at least 2 dimensions');
end

if dimension_map < 2
    error('Obstacles must have at least 2 dimensions');
end

for i = 1: length(arena_map)
    if size(arena_map{i},2) ~= dimension_start
        error('All obstacles and robot configuration must have the same dimension');
    end
end

for i = 1: length(arena_map)
    if size(arena_map{i},2) ~= dimension_goal
        error('All obstacles and goal configuration must have the same dimension');
    end
end

if (dimension_start ~= 2 || dimension_goal ~= 2 || dimension_map ~= 2 ) && obst_approx == "EXACT"
    error('Exact obstacle representation is only available for 2D configuration spaces');
end

end