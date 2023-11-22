function [path] = attr_repl( qstart, qgoal )
%ATTR_REPL Additive attraction and repulsion path planning implementation
%  This function implements the additive attraction and repulsion method
%  for path planning. The function takes as input the start and goal
%  positions of the robot and returns the path as an n dimensional vector.
%  The number of robot and the goal is assumed to be 1.
%       
%   - Input:  qstart = [s_1 s_2 ... s_n]     starting coordinates of the robot,
%             qgoal  = [g_1 g_2 ... g_n]     goal coordinates of the robot,
%                                            where "n" is the dimension of the
%                                            configuration space.
%
%   - Output: path  =  [p_11 p_21 ... p_n1;  path of the robot, where rows are the
%                       p_12 p_22 ... p_n2;  coordinates of the robot at a given
%                       ...                  time step and "m" is the number of
%                       p_1m p_2m ... p_nm]  steps.
%                                            
%                                            
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023 

% Extern global variables
global sensor_range ; 
global infinity ;
global arena_map ;
global solver ;
global dimension ;

    % Initial declarations
    path = zeros(dimension, 1);     % Path is initialized with only 1 step




    % Attractive Potential Gradient
    for n = 1: dimension
        path(n, 1) = qstart(n) + attrGrad(qstart, qgoal);
        
    end

    end



    if solver == "DISCRETE"
        % discrete method
        [x,y] = discrete(qstart, qgoal);
    end







    ref_dist = sensor_range*0.5;
    circumnav_dir = -1;   % CCW = 1, CW = -1
    status = 3;  
    step = 0.02; 


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

        if (dist<=ref_dist)
            % if it is the first encounter, save the point of contact.
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

        % if the distance between the point of contact of the new obstacle is in close proximity of the closest point of the previous obstacle, then there is no solution
        if abs(x_encounter(obstacle_num) - minimum_point{obstacle_num-1}(1)) < epsilon && abs(y_encounter(obstacle_num) - minimum_point{obstacle_num-1}(2)) < epsilon
            disp('no solution')
            break;
        end

        % leave lock is to ensure that the robot has gotten far enough from the point of contact initially before checking that it is circumnavigated.
        % this could also be implemented in counting for a fixed amount of steps before checking for circumnavigation.
        if (abs(x_encounter(obstacle_num) - x(i)) > 3*epsilon ) || (abs(y_encounter(obstacle_num) - y(i)) > 3*epsilon)
            leave_lock = 0;
        end

        % closest point to the goal of the current obstacle is saved in terms of global coordinates.
        if (norm([qgoal(1)-x(i) qgoal(2)-y(i)]) < minimum_dist(obstacle_num)) && (leave == 0) && (status == 1)
            minimum_dist(obstacle_num)  = norm([qgoal(1)-x(i) qgoal(2)-y(i)]);
            minimum_point{obstacle_num} = [x(i) y(i)];
        end

        if (abs(x_encounter(obstacle_num) - x(i)) < 2*epsilon) && (abs(y_encounter(obstacle_num) - y(i)) < 2*epsilon)
            % if the robot has circumnavigated the obstacle, and the leave lock is off, then the robot can go to the closest point on the obstacle to the goal.
            % leave variable holds the information of whether the robot has circumnavigated the obstacle or not. it is ready to leave when leave = 1.
            if leave_lock == 0
                leave = 1;
            end
        end

        % finally, if the robot has reached the closest point to the goal and the leave is initiated, then the robot can go to the goal.
        if (abs(x(i) - minimum_point{obstacle_num}(1)) < epsilon) && (abs(y(i) - minimum_point{obstacle_num}(2)) < epsilon) && leave == 1
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

        % time increment.
        i = i + 1;
    end

end

