function [x,y] = bug_fast_planner( qstart, qgoal )

global sensor_range infinity arena_map ;

% % A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
% 
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));

% samples = norm( qgoal - qstart ) / (sensor_range / 20);
% range = linspace(0,1,samples);
ref_dist = 0.25;
circumnav_dir = -1;   % CCW = 1, CW = -1
status = 1;
range = 0.075;
x(1) = qstart(1);
y(1) = qstart(2);

for i=2:1000
    [dist, min]= rps_sensor(arena_map, [x(i-1) y(i-1)]);
    if dist > sensor_range
        dist = sensor_range;
    end
    
    if (dist<=0.4)|| (dist>=0.15)
        status = 1;
    end
    if (dist<0.15)
        status = 2;
    end
    if (dist> 0.4)
        status = 3;
    end
    % move in the direction orthogonal to the minimum distance
    % tangent move
    if status == 1
        x(i) = x(i-1) + range*1*(cos(min-(pi/2*circumnav_dir))) + range*1*(cos(min))*(dist-ref_dist);
        y(i) = y(i-1) + range*1*(sin(min-(pi/2*circumnav_dir))) + range*1*(sin(min))*(dist-ref_dist);
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

