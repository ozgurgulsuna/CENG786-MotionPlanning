function [x,y] = bug_one_fast( qstart, qgoal )

global sensor_range infinity arena_map ;

% % A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
% 
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));

% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
ref_dist = sensor_range*0.5;
circumnav_dir = -1;   % CCW = 1, CW = -1
status = 1;
range = 0.075;
x(1) = qstart(1);
y(1) = qstart(2);

for i=1:200
    [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);

    if (dist<=sensor_range) || (dist>=sensor_range*0.2)
        status = 1;
    end
    if (dist<sensor_range*0.2)
        status = 2;
    end
    if (dist > sensor_range)
        status = 3;
    end

    % move in the direction orthogonal to the minimum distance
    % tangent move
    if status == 1
        x(i+1) = x(i) + range*1*(cos(min-(pi/2*circumnav_dir))) + range*5*(cos(min))*(dist-ref_dist);
        y(i+1) = y(i) + range*1*(sin(min-(pi/2*circumnav_dir))) + range*5*(sin(min))*(dist-ref_dist);
    end

    % get away
    if status == 2
        x(i+1) = x(i) - range*0.5*(cos(min));
        y(i+1) = y(i) - range*0.5*(sin(min));
    end

    if status == 3
        x(i+1) = x(i) + range*0.5*(qgoal(1)-x(i));
        y(i+1) = y(i) + range*0.5*(qgoal(2)-y(i));
    end
    
end

