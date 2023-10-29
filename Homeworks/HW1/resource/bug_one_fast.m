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
    status = 3;
    step = 0.02;
    x(1) = qstart(1);
    y(1) = qstart(2);

    i = 1;
    encounter = 0;
    obstacle_num = 0;
    minimum_point = [1000 1000];
    minimum_dist = infinity;
    x_encounter = 1000;
    y_encounter = 1000;
    leave = 0;

    % minimum distance to the critical points, if we increase the fineness of the
    % path, i.e. the number of points using step, we can decrease this further.
    % as a rule of thumb 5 times is a good start.
    epsilon = step*5; 


    while (norm([x(i) y(i)]-qgoal) > 0.1)
    % for i=1:3000
        [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);
        % if (dist > sensor_range)
        %     % move to the goal
        %     status = 3; 
        %     encounter = 0; 
        % end

        if (dist<=ref_dist)
            % if it is the first encounter, save the point of contact
            if (encounter == 0)
                obstacle_num = obstacle_num + 1
                x_encounter = x(i);
                y_encounter = y(i);
                encounter = 1;
                leave_lock = 1;
                status = 1;
                leave = 0;
            end
            % circumnavigate the obstacle
            

        end

        if (abs(x_encounter - x(i)) > 3*epsilon ) || (abs(y_encounter - y(i)) > 3*epsilon)
            leave_lock = 0;
        end


        % record the minimum distance
        if (norm([qgoal(1)-x(i) qgoal(2)-y(i)]) < minimum_dist) && (leave == 0)
            minimum_dist = norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            minimum_point = [x(i) y(i)];
        end

        if (abs(x_encounter - x(i)) < 2*epsilon) && (abs(y_encounter - y(i)) < 2*epsilon)
            % if the robot has circumnavigated the obstacle, go to the minimum distance point
            if leave_lock == 0
                leave = 1;
            end
        end

        if (abs(x(i) - minimum_point(1)) < epsilon) && (abs(y(i) - minimum_point(2)) < epsilon) && leave == 1
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


% for i=1:200
%     [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);

%     if (dist<=sensor_range) || (dist>=sensor_range*0.2)
%         status = 1;

%     end
%     if (dist<sensor_range*0.2)
%         status = 2;
%     end
%     if (dist > sensor_range)
%         status = 3;
%     end

%     % move in the direction orthogonal to the minimum distance
%     % tangent move
%     if status == 1
%         x(i+1) = x(i) + range*1*(cos(min-(pi/2*circumnav_dir))) + range*5*(cos(min))*(dist-ref_dist);
%         y(i+1) = y(i) + range*1*(sin(min-(pi/2*circumnav_dir))) + range*5*(sin(min))*(dist-ref_dist);
%     end

%     % get away
%     if status == 2
%         x(i+1) = x(i) - range*0.5*(cos(min));
%         y(i+1) = y(i) - range*0.5*(sin(min));
%     end

%     if status == 3
%         x(i+1) = x(i) + range*0.5*(qgoal(1)-x(i));
%         y(i+1) = y(i) + range*0.5*(qgoal(2)-y(i));
%     end
    
% end

