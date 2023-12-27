function check = checkPath(current_configuration, next_configuration)
%CHECKPATH the function checks if the local path between two configurations
%   is collision free. For the collision checking, the function interpolates
%   the path between two configurations with N steps and checks if any of
%   the interpolated configurations is in collision with the obstacles. If
%   there is a collision the function is halted and returns 1.
%   TODO: perhaps change the incremental method to subdivision method
%   
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map sample;

N = 10; % number of steps for interpolation
check = 0; % return value

% interpolate the path between two configurations
for i = 1:size(map.obstacles,2)
    if any(any(intersect(map.obstacles{i}, [current_configuration(1) current_configuration(2) ; next_configuration(1) next_configuration(2)])))
        check = 1;
        return;
    end
end


for i = 1:N
    q = (N-i)/N*current_configuration + i/N*next_configuration;
    % createRobot(q,"draw");
    if checkCollision(q)
        check = 1;
        return;
    end
end

end
