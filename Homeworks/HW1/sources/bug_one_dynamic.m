function [x,y] = bug_one_fast( qstart, qgoal )
global sensor_range infinity arena_map ;

    ref_dist = sensor_range*0.5;
    circumnav_dir = -1;   % CCW = 1, CW = -1
    status = 3;
    count = 0;
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

    meeting_point = [0 0];


    while (norm([x(i) y(i)]-qgoal) > epsilon/3)
    %for i=1:1500
        [dist, min]= rps_sensor(arena_map, [x(i) y(i)]);
        % if (dist > sensor_range)
        %     % move to the goal
        %     status = 3; 
        %     encounter = 0; 
        % end
        if (status == 3) && (count == 0)
            if (dist<=ref_dist)
                % if it is the first encounter, save the point of contact.
                if (encounter == 0)
                    obstacle_num = obstacle_num + 1;
                    x_encounter(obstacle_num) = x(i)-meeting_point(1);
                    y_encounter(obstacle_num) = y(i)-meeting_point(2);
                    encounter = 1;
                    leave_lock = 1;
                    status = 1;
                    leave = 0;
                    minimum_dist(obstacle_num) = infinity;
                    count = 5;
                end
            end
        elseif (status == 3) 
            count = count - 1;
        end

        % % No solution check for dynamic case should be handled differently
        % % because the goal reacheability condition can change, i.e. the obstacle move away.
        % if abs(x_encounter(obstacle_num) - minimum_point{obstacle_num-1}(1)) < epsilon && abs(y_encounter(obstacle_num) - minimum_point{obstacle_num-1}(2)) < epsilon
        %     disp('no solution')
        %     break;
        % end

        % leave lock is to ensure that the robot has gotten far enough from the point of contact initially before checking that it is circumnavigated.
        % this could also be implemented in counting for a fixed amount of steps before checking for circumnavigation.
        if (abs(x_encounter(obstacle_num) - x(i)+meeting_point(1)) > 10*epsilon ) || (abs(y_encounter(obstacle_num) - y(i)+meeting_point(2)) > 10*epsilon)
            leave_lock = 0;
        end

        % closest point to the goal of the current obstacle is saved in terms of global coordinates.
        if (norm([qgoal(1)-x(i) qgoal(2)-y(i)]) < minimum_dist(obstacle_num)) && (leave == 0) && (status == 1)
            minimum_dist(obstacle_num)  = norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            minimum_point{obstacle_num} = [(x(i)-meeting_point(1)) (y(i)-meeting_point(2))];
        end

        
        if (abs(x_encounter(obstacle_num) - x(i)+meeting_point(1)) < epsilon) && (abs(y_encounter(obstacle_num) - y(i)+meeting_point(2)) < epsilon) && (leave == 0)
            % if the robot has circumnavigated the obstacle, and the leave lock is off, then the robot can go to the closest point on the obstacle to the goal.
            % leave variable holds the information of whether the robot has circumnavigated the obstacle or not. it is ready to leave when leave = 1.
            if leave_lock == 0
                leave = 1;
            end
        end
         
        % finally, if the robot has reached the closest point to the goal and the leave is initiated, then the robot can go to the goal.
        if (abs(x(i) - minimum_point{obstacle_num}(1)-meeting_point(1)) < 2*epsilon) && (abs(y(i) - minimum_point{obstacle_num}(2)-meeting_point(2)) < 2*epsilon) && (leave == 1)
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
            x(i+1) = x(i) + 2*step*(qgoal(1)-x(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            y(i+1) = y(i) + 2*step*(qgoal(2)-y(i))/norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
        end


        
        % move arena map at a fixed rate
        for j=1:size(arena_map,2)
            arena_map{j}(:,1) = arena_map{j}(:,1) + step*0.1;
            arena_map{j}(:,2) = arena_map{j}(:,2) + step*0.0018;
        end

        % update the meeting point of the robot and the obstacle. (a way to track the movement of obstacles)
        meeting_point(1) = meeting_point(1) + step*0.1;
        meeting_point(2) = meeting_point(2) + step*0.0018;
        
        % time increment.
        i = i + 1;

    end
    display('done')

end

