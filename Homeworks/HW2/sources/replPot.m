function [Urepl] = replPot(q, obst_num)
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

eta = 3;  % Repulsive potential gain
Q_star = 20 ; % Distance at which the potential is truncated

% Compute the distance from the robot to the obstacle
[dista, angle ] = rps_sensor(q, obst_num) ;


if dista <= Q_star
    Urepl = 0.5 * eta * (1/dista - 1/Q_star)^2 ;
else
    Urepl = [0 ; 0] ;
end

    

end