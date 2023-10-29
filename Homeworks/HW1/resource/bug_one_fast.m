function [x,y] = bug_one_fast( qstart, qgoal )
global sensor_range infinity arena_map ;

    ref_dist = sensor_range*0.5;
    circumnav_dir = -1;   % CCW = 1, CW = -1
    status = 3;
    step = 0.02;
    x(1) = qstart(1);
    y(1) = qstart(2);

    i = 1;
    encounter = 0;
    obstacle_num = 2;
    minimum_point{1} = [infinity*2 infinity*2];
    minimum_dist(1) = infinity;
    x_encounter(1) = infinity;
    y_encounter(1) = infinity;

    minimum_point{2} = [infinity*2 infinity*2];
    minimum_dist(2) = infinity;
    x_encounter(2) = infinity;
    y_encounter(2) = infinity;

    leave = 0;

    % minimum distance to the critical points, if we increase the fineness of the
    % path, i.e. the number of points using step, we can decrease this further.
    % as a rule of thumb 5 times is a good start.
    epsilon = step*5; 


    while (norm([x(i) y(i)]-qgoal) > epsilon/3)
    %for i=1:3000
        [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);
        % if (dist > sensor_range)
        %     % move to the goal
        %     status = 3; 
        %     encounter = 0; 
        % end

        if (dist<=ref_dist)
            % if it is the first encounter, save the point of contact
            if (encounter == 0)
                obstacle_num = obstacle_num + 1;
                x_encounter(obstacle_num) = x(i);
                y_encounter(obstacle_num) = y(i);
                encounter = 1;
                leave_lock = 1;
                status = 1;
                leave = 0;
                minimum_dist(obstacle_num) = infinity;
            end

        end

        if abs(x_encounter(obstacle_num) - minimum_point{obstacle_num-1}(1)) < epsilon && abs(y_encounter(obstacle_num) - minimum_point{obstacle_num-1}(2)) < epsilon
            disp('no solution')
            break;
        end


        if (abs(x_encounter(obstacle_num) - x(i)) > 3*epsilon ) || (abs(y_encounter(obstacle_num) - y(i)) > 3*epsilon)
            leave_lock = 0;
        end

        % record the minimum distance
        if (norm([qgoal(1)-x(i) qgoal(2)-y(i)]) < minimum_dist(obstacle_num)) && (leave == 0) && (status == 1)
            minimum_dist(obstacle_num)  = norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            minimum_point{obstacle_num} = [x(i) y(i)];
        end

        if (abs(x_encounter(obstacle_num) - x(i)) < 2*epsilon) && (abs(y_encounter(obstacle_num) - y(i)) < 2*epsilon)
            % if the robot has circumnavigated the obstacle, go to the minimum distance point
            if leave_lock == 0
                leave = 1;
            end
        end

        if (abs(x(i) - minimum_point{obstacle_num}(1)) < epsilon) && (abs(y(i) - minimum_point{obstacle_num}(2)) < epsilon) && leave == 1
            % if the robot has reached the minimum distance point, go to the goal
            status = 3;
            encounter = 0;
        end
        

        if status == 1
            x(i+1) = x(i) + step*1*(cos(min-(pi/2*circumnav_dir))) + step*1*(cos(min))*(dist-ref_dist);
            y(i+1) = y(i) + step*1*(sin(min-(pi/2*circumnav_dir))) + step*1*(sin(min))*(dist-ref_dist);
        end


        if status == 3
            x(i+1) = x(i) + step*(qgoal(1)-x(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            y(i+1) = y(i) + step*(qgoal(2)-y(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
        end

        i = i + 1;
    end

end

