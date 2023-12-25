function robot = createRobot(configuration,draw)
%CREATEROBOT Creates a robot object with the given configuration.
%   The configuration is a vector of N+2 elements, where N is the number of
%   arms. The first two elements are the x and y coordinates of the base of
%   the robot. The next N elements are the angles of the arms with respect
%   to the previous arm, in radians.
%
%   The robot object is a struct with the following fields:
%   - base: a 2x1 vector with the x and y coordinates of the base of the
%   robot.
%   - angles: a Nx1 vector with the angles of the arms with respect to the
%   previous arm, in radians.
%   - lengths: a Nx2 matrix with the lengths of each arm. The first column
%   is the length of the first segment of the arm, and the second column is
%   the length of the second segment of the arm.
%   - closed: a boolean indicating whether the robot is closed or not.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

lengths = [1 15 ; 1 15 ; 1 5 ]; % Length of each arm

% check if the configuration is valid
if length(configuration) ~= 2 + length(lengths)
    error('Invalid configuration');
end

% create the robot object
robot = struct('base', [configuration(1); configuration(2)], ...
               'angles', configuration(3:end), ...
               'lengths', lengths, ...
               'bodies', [], ...
               'closed', false);

% make sure the angles are between 0 and 2*pi
for i = 1:length(lengths)
    robot.angles(i) = mod(robot.angles(i), 2*pi);
end

% create each arm
next_base = robot.base;
previous_angle = 0;
for i = 1:length(lengths)
    corner_1 = [next_base(1) + robot.lengths(i,1)*sin(robot.angles(i)+previous_angle), next_base(2) - robot.lengths(i,1)*cos(robot.angles(i)+previous_angle)];
    corner_2 = [2*next_base(1)-corner_1(1),2*next_base(2)-corner_1(2)];
    corner_3 = [corner_2(1) + robot.lengths(i,2)*cos(robot.angles(i)+previous_angle),corner_2(2) + robot.lengths(i,2)*sin(robot.angles(i)+previous_angle)];
    corner_4 = [corner_1(1) + robot.lengths(i,2)*cos(robot.angles(i)+previous_angle),corner_1(2) + robot.lengths(i,2)*sin(robot.angles(i)+previous_angle)];
    next_base = [(corner_3(1)+corner_4(1))/2,(corner_3(2)+corner_4(2))/2];
    previous_angle = robot.angles(i)+previous_angle;
    robot.bodies = [robot.bodies; [corner_1(1) corner_2(1) corner_3(1) corner_4(1)] [corner_1(2) corner_2(2) corner_3(2) corner_4(2)]];

    if draw =="draw"
        newcolors = [[0 0.4470 0.7410] ; [0.8500 0.3250 0.0980] ; [0.9290 0.6940 0.1250] ; [0.4940 0.1840 0.5560] ; [0.4660 0.6740 0.1880] ; [0.3010 0.7450 0.9330] ; [0.6350 0.0780 0.1840]];
        c = newcolors(mod(i,7)+1,:);
    
        % draw the arm
        patch([corner_1(1), corner_2(1), corner_3(1), corner_4(1)], ...
            [corner_1(2), corner_2(2), corner_3(2), corner_4(2)],c);
        hold on;
    end

end

end

