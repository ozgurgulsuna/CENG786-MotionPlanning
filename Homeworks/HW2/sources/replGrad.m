function [nabla_Urepl] = replGrad(q, obst_num)
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

eta = 0.003;  % Repulsive potential gain
Q_star = 20 ; % Distance at which the potential is truncated

% Compute the distance from the robot to the obstacle
[dista, angle ] = rps_sensor(q, obst_num) ;


if dista <= Q_star
    nabla_Urepl = eta * (1/dista - 1/Q_star) * (1/dista^2) * [cos(angle) ; sin(angle)] ;
else
    nabla_Urepl = [0 ; 0] ;
end

    

end