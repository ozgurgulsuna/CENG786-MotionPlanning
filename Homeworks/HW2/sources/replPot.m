function [Urepl] = replPot(q, obst_num)
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global obst_approx approx_map;

eta = 0.5;  % Repulsive potential gain
Q_star = 20 ; % Distance at which the potential is truncated

% Compute the distance from the robot to the obstacle
if obst_approx == "EXACT"
    [dist, angle ] = rps_sensor(q, obst_num) ;
    if dist <= Q_star
        Urepl = 0.5 * eta * (1/dist - 1/Q_star)^2 ;
    else
        Urepl = [0 ; 0] ;
    end

elseif obst_approx == "APPROX"
    % center point of the obstacle
    v = (approx_map{obst_num}(1,:)+approx_map{obst_num}(2,:))/2 ;

    % distance from the robot to the center of the n-dimensonal coordinate of the obstacle
    dist = norm(q-v) ;

    % radius of the obstacle
    radius = norm(approx_map{obst_num}(1,:) - approx_map{obst_num}(2,:))/2 ;

    % distance from to the closest point on the n-sphere
    dist_c = dist - radius ;

    % closest point on the n-sphere
    c = v + (q-v)*radius/dist ;

    if dist_c <= Q_star
        Urepl = 0.5 * eta * (1/dist_c - 1/Q_star)^2 ;
    else
        Urepl = zeros(length(q),1) ;
    end

end
end
