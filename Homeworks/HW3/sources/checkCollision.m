function check = checkCollision(configuration)
%CHECKCOLLISION Checks if the configuration is in the free space.
%   There are two collisions to check: collision with the obstacles and
%   collision with the outer boundary of the environment.
%   The function returns 1 if there is a collision and 0 otherwise.

global map;

robot = createRobot(configuration,"draw");

% Check collision with the obstacles
for i = 1:size(map.obstacles, 2)
    for j = 1:size(robot.bodies, 1)
        arm = polyshape([robot.bodies(j,1:4)],[robot.bodies(j,5:8)]);
        obstacle = map.obstacles{i};
        if overlaps(arm, obstacle)
            % collision detected
            check = 1; 
            return;
        elseif overlaps(arm, map.outside)
            % out of bounds
            check = 1;
            return;
        else
            check = 0;
        end
    end
end
