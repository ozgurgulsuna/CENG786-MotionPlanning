function [x,y] = bug_two( qstart, qgoal )
global sensor_range infinity arena_map ;

    ref_dist = sensor_range*0.5;
    circumnav_dir = -1;   % CCW = 1, CW = -1
    status = 3;
    step = 0.02;
    x(1) = qstart(1);
    y(1) = qstart(2);

    i = 1;
    encounter = 0;
    obstacle_num = 1;

    x_encounter(1) = infinity;
    y_encounter(1) = infinity;

    % minimum distance to the critical points, if we increase the fineness of the
    % path, i.e. the number of points using step, we can decrease this further.
    % as a rule of thumb 5 times is a good start.
    epsilon = step*5; 


    while (norm([x(i) y(i)]-qgoal) > epsilon/3)
    %for i=1:1500
        [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);

        if (dist<=ref_dist)
            % if it is the first encounter, save the point of contact.
            if (encounter == 0)
                x_encounter(obstacle_num) = x(i);
                y_encounter(obstacle_num) = y(i);
                encounter = 1;
                status = 1;
            end

        end

        % if the robot hits the m-line, head towards to the goal.
        [distance, ~] = point_segment_distance([x(i) y(i)], [qstart; qgoal]);
        if (distance < 2*epsilon) && ((norm([x(i) y(i)]-qgoal))< (norm([x_encounter(obstacle_num) y_encounter(obstacle_num)]-qgoal)))
            status = 3;
            encounter = 0;
        end
        
        % main control loop for the boundary following, calculates the next step.
        if status == 1
            x(i+1) = x(i) + step*1*(cos(min-(pi/2*circumnav_dir))) + step*1*(cos(min))*(dist-ref_dist);
            y(i+1) = y(i) + step*1*(sin(min-(pi/2*circumnav_dir))) + step*1*(sin(min))*(dist-ref_dist);
        end

        % moving towards the goal with a fixed step size.
        if status == 3
            x(i+1) = x(i) + step*(qgoal(1)-x(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            y(i+1) = y(i) + step*(qgoal(2)-y(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
        end

        % move arena map at a fixed rate
        for j=1:size(arena_map,2)
            arena_map{j}(:,1) = arena_map{j}(:,1) + step*0.1;
            arena_map{j}(:,2) = arena_map{j}(:,2) + step*0.0018;
        end

        % time increment.
        i = i + 1;
    end

end

function [ distance, angle ] = point_segment_distance( v , x )
    % POINT_SEGMENT_DISTANCE Calculate the distance between a point and a line segment.
    %
    %   - Input: v = [x y] (robot position)
    %            ab = [[x1 y1] [x2 y2]] (edge)
    %
    %   - Output: closest_vector = [x y] (closest point on the edge)

    % get the x and y values of the first vertex
    a_x = x(1, 1);
    a_y = x(1, 2);

    % get the x and y values of the second vertex
    b_x = x(2, 1);
    b_y = x(2, 2);

    ab = [b_x - a_x, b_y - a_y];
    av = [v(1) - a_x, v(2) - a_y];

    % calculate the dot product of ab and av
    av_ab = dot(av, ab);

    % length square of ab
    % ab_ab = dot(ab, ab);
    ab_ab = norm(ab)^2;

    % distance then is
    d = av_ab / ab_ab;

    if d < 0
        closest_point = [a_x, a_y];
    elseif d >= 1
        closest_point = [b_x, b_y];
    else
        closest_point = [a_x + ab(1) * d, a_y + ab(2) * d];
    end

    distance = sqrt((v(1) - closest_point(1))^2 + (v(2) - closest_point(2))^2);
    angle = atan2(closest_point(2) - v(2), closest_point(1) - v(1));

    if angle < 0
        angle = angle + 2 * pi;
    end
    
end