function [ dist, angle ] = calculate_distance_angle( visible_edges, v )
    % calculate the minimum distance and angle from given visible edges and robot position
    %
    %   - Input: visible_edges = [[x1 y1; x2 y2], [x1 y1; x2 y2], ...]
    %                    v = [x y] (robot position)
    %
    %   - Output: dist = minimum distance
    %             angle = angle of the minimum distance

    % initialize the minimum distance
    dist = 1000000;

    % initialize the angle of the minimum distance
    angle = 0;

    % traverse the visible edges
    for i = 1: length(visible_edges)

        % get the x and y values of the first vertex
        x_1 = visible_edges{i}(1, 1);
        y_1 = visible_edges{i}(1, 2);

        % get the x and y values of the second vertex
        x_2 = visible_edges{i}(2, 1);
        y_2 = visible_edges{i}(2, 2);

        % calculate the distance between the robot position and the edge
        [ distance, ang ] = point_segment_distance( [v(1) v(2)],[[ x_1 y_1];[ x_2 y_2]] ); % second part is just visible_edges{i}

        % if the distance is less than the minimum distance
        if distance < dist

            % update the minimum distance
            dist = distance
            angle = ang

        end

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