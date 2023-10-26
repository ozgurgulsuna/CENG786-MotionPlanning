function [x,y] = bug_planner( qstart, qgoal )

global sensor_range infinity;

% % A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
% 
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));

% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
status = 1;
range = 0.1;
x(1) = qstart(1);
y(1) = qstart(2);
for i=2:200
    dist= [10 5];
    min = 0; % angle of minimum distance
    % sweep the sensor around
    for theta=0:2*pi/50:2*pi  % 10 degree steps
        dist(2) = read_sensor(theta, [x(i-1) y(i-1)]);
        if (dist(2)<dist(1))
            dist(1) = dist(2) ;
            min = theta;
            status = 1;
        end
        if (dist(1)<0.10)
            status = 2;
        end
        if (dist(1)> 0.4)
            status = 3;
        end

    end

    % move in the direction orthogonal to the minimum distance
    % tangent move
    if status == 1
        x(i) = x(i-1) + range*1*(cos(min+pi/2));
        y(i) = y(i-1) + range*1*(sin(min+pi/2));
    end

    % get away
    if status == 2
        x(i) = x(i-1) - range*0.5*(cos(min));
        y(i) = y(i-1) - range*0.5*(sin(min));
    end
    % get in
    if status == 3
        x(i) = x(i-1) + range*0.5*(qgoal(1)-x(i-1));
        y(i) = y(i-1) + range*0.5*(qgoal(2)-y(i-1));
    end
    status;
end
