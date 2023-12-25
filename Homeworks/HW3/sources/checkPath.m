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


N = 10; % number of steps for interpolation
check = 0; % return value

% interpolate the path between two configurations
for i = 1:N
    q = (N-i)/N*current_configuration + i/N*next_configuration;
    % createRobot(q,"draw");
    if checkCollision(q)
        check = 1;
        break;
    end
end

end
