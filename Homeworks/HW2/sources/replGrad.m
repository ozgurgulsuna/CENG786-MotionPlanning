function nabla_Urepl = replGrad(q, obst_num)
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

eta = 0.1 ;  % Repulsive potential gain
Q_star = 1 ; % Distance at which the potential is truncated

% Compute the distance from the robot to the obstacle
[dist, angle ] = rps_sensor(q, obst_num) ;


if dist <= Q_star
    nabla_Urepl = eta * (1/dist - 1/Q_star) * (1/dist^2) * [cos(angle) ; sin(angle)] ;
else
    nabla_Urepl = [0 ; 0] ;
end

    

end