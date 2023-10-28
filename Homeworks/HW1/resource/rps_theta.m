function [ dist, angle ] = rps_theta( arena_map , v)
global closest_vec;
    % (RPS) Rotational Plane Sweep Algorithm, find the visibility graph from given vertices
    % then generate new unified simple obstacle map from visible edges.
    %
    %   - Input: arena_map = [[x1 y1; x2 y2; ... ; xn yn], [x1 y1; x2 y2; ... ; xm ym], ...]
    %                    v = [x y] (robot position)
    %
    %   - Output: visible_edges = [[x1 y1; x2 y2], [x1 y1; x2 y2], ...]
    %
    % Note: robot position (v) being inside the obstacle is handled outside of this function.

        % create vertices from given arena_map
        [ vertices ] = generate_vertices( arena_map );

        % number of vertices
        N = size(vertices, 1);

        % add the robot position as the last vertex with identifier "0"
        vertices(N+1,:) = [v, 0];

        % create the list of edges according to the vertices
        [ E ] = calculate_edges( vertices, N );

        % draw the graph
        % draw_graph( vertices, E );

        % initialize visibility graph
        visibility_graph = [];

        % initialize active list
        [ S ] = zeros(N, N);

        % initialize distance and angle of the closest vector
        closest_vec = [[0 0], [0 0], 1000000, 0];

        % evaluated starting vertex (v), or robot position (last vertex)
        % TODO: if v is inside the obstacle, then it is not evaluated
        % to create complete visibility graph, traverse all vertices (i = 1 : N)
        for i = N + 1

            % calculate the angles of the vertices
            [ A ] = calculate_angle( i, vertices, N );

            % initialize active list
            [ S ] = intersects_line( i, vertices, E, S );

            % traverse the angles up to the last vertex angle (some vertices may have the same angle)
            for j = 1: size(A, 1)
                
                % initial vertex
                vertex = i ;

                % get the vertex index
                vertex_j = A(j, 3);

                % check intersection between vertex and vertex_j
                [ draw_line, adjacency_list ] = check_intersection( vertex, vertex_j, vertices, E, S );

                % if there is no intersection, draw the line
                if draw_line == 1

                    % draw the line
                    node_1 = [vertices(vertex, 1) vertices(vertex, 2)];
                    node_2 = [vertices(vertex_j, 1) vertices(vertex_j, 2)];
                    % line([node_1(1, 1), node_2(1, 1)], [node_1(1, 2), node_2(1, 2)], 'Color', 'r');

                    % add the edge to the visibility graph
                    visibility_graph = [visibility_graph; vertex vertex_j];
                end

                % update active list
                [ S ] = check_active_list( vertex, vertex_j, vertices, adjacency_list, S );

            end

            % clear active list
            S = zeros(N, N);
        end


        [h w] = size(visibility_graph);
    
        % traverse the all vertices
        for k = 1: h
    
            i = visibility_graph(k, 1);
            j = visibility_graph(k, 2);
    
            edges(i, j) = 1;
            edges(j, i) = 1;
    
        end
    
        [h w] = size(edges);
    
        result = [];
    
        % traverse the edges
        for i = 1: h
    
            for j = 1: i
    
                if edges(i, j) ~= 0
    
                    % edges = [edges; edges(i, j) i j];
                    result = [result; j i];
    
                end
    
            end
    
        end
    
        edges = sortrows(result);
        dist = closest_vec(5);
        angle = closest_vec(6);
        
        % create new unified simple obstacle map from visible edges
        %[ visible_edges ] = generate_visible_edges( arena_map, vertices, visibility_graph, E );

        % calculate the minimum distance and angle
        %[ dist, angle ] = calculate_distance_angle( visible_edges, v );

end


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
            dist = distance;
            angle = ang;

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

function [ visible_edges ] = generate_visible_edges( arena_map, vertices, visibility_graph, edges )
    % generate new unified simple obstacle map from visible edges.
    %
    %   - Input: arena_map = [[x1 y1; x2 y2; ... ; xn yn], [x1 y1; x2 y2; ... ; xm ym], ...]
    %            edges = [[x1 y1; x2 y2], [x1 y1; x2 y2], ...]
    %            visibility_graph = [[x1 y1; x2 y2], [x1 y1; x2 y2], ...]
    %
    %   - Output: visible_edges = [[x1 y1; x2 y2], [x1 y1; x2 y2], ...]

    % visible_edges = zeros(length(visibility_graph(:,2)), 4);
    visible_edges = {};

    len = length(visibility_graph(:,2));
    for i = 1: len
        idx = visibility_graph(i,2);
        [h w] = size(edges);
        % draw edges
        for j = 1: w
        
            % check there is an edge between vertices or not
            if edges(idx, j) == 1
                
                % get the x and y values of the first vertex
                x_1 = vertices(idx, 1);
                y_1 = vertices(idx, 2);

                % get the x and y values of the second vertex
                x_2 = vertices(j, 1);
                y_2 = vertices(j, 2);

                % line([x_1, x_2], [y_1, y_2], 'Color', 'b');

                visible_edges{end+1} = [x_1 y_1; x_2 y_2];
            
            end
            
        end

        
        % % number of edges
        % N = size(edges, 1);

        % % initialize visible edges
        % visible_edges = [];

        % % traverse the edges
        % for i = 1: N

        %     % get the x and y values of the first vertex
        %     x_1 = edges(i, 1);
        %     y_1 = edges(i, 2);

        %     % get the x and y values of the second vertex
        %     x_2 = edges(i, 3);
        %     y_2 = edges(i, 4);

        %     visibility_graph(:,2)

        %     % check whether the edge is in the visibility graph or not
        %     if ismember([x_1 y_1; x_2 y_2], visibility_graph, 'rows') == 1

        %         % add the edge to the visible edges
        %         visible_edges = [visible_edges; x_1 y_1; x_2 y_2];

        %     end

        % end

        % % number of visible edges
        % N = size(visible_edges, 1);

        % % traverse the visible edges
        % for i = 1: N

        %     % get the x and y values of the first vertex
        %     x_1 = visible_edges(i, 1);
        %     y_1 = visible_edges(i, 2);

        %     % get the x and y values of the second vertex
        %     x_2 = visible_edges(i + 1, 1);
        %     y_2 = visible_edges(i + 1, 2);

        %     % check whether the edge is in the visibility graph or not
        %     if ismember([x_1 y_1; x_2 y_2], visibility_graph, 'rows') == 1

        %         % remove the edge from the visible edges
        %         visible_edges(i + 1, :) = [];

        %     end

        % end

        % % number of visible edges
        % N = size(visible_edges, 1);

    end
end

function [ E ] = calculate_edges( vertices , N)
    % CALCULATE_EDGES Transforms the initial set of vertices into a data
    % structure that represents the edges of the polygons.
    %
    %   E: Array of size M x 2, where M is the number of edges. Each row
    %   represents one edge, having the start and the end vertices index of the
    %   array of vertices.
    %
    %   vertices: Array of size N x 3, where N is the number of vertices. Each
    %   row represents one vertex, having the x coordinate, y coordinate and
    %   the object number.
    %
    %   N: Number of vertices

        % initialization of the variables
        E = zeros(N, N);   
        edge_idx = 1;
        init_vertex_idx = 1;     
    
        % create edges
        for i = 1: N-1

            % check object number of the current vertex and the next vertex
            if vertices(i, 3) == vertices(i + 1, 3)
                % if they are the same, create an edge between them 
                % TODO: we can name the edges according to the object number
                E(i, i + 1) = 1;
                E(i + 1, i) = 1;
    
            else
                % if they are not the same, create an edge between the first vertex 
                E(i, init_vertex_idx) = 1;
                E(init_vertex_idx, i) = 1;
                init_vertex_idx = i + 1;
    
            end
        end
        % add the connection between the last vertex and the first vertex of the last object
        E(i+1,init_vertex_idx) = 1;
        E(init_vertex_idx,i+1) = 1;
end

function [ A ] = calculate_angle( v, vertices, N )
    % CALCULATE_ANGLE Calculate the angle of the each vertices with respect to
    % the vertex at index i. The angles are sorted in ascending order.
    %
    %   A: Array of size N x 3, where N is the number of vertices. Each row
    %   represents one vertex, having the angle, the angle modulo 2*pi and the
    %   index of the vertex in the array of vertices.
    %
    %   v: Index of the vertex with respect to which the angles are calculated.
    %
    %   vertices: Array of size N x 3, where N is the number of vertices. Each
    %   row represents one vertex, having the x coordinate, y coordinate and
    %   the object number which represents the polygon to which the vertex
    %   belongs.
    %
    %   N: Number of vertices
    
        % initialize angles
        A = [];

        % calculate the angles of the all vertices
        for i = 1: N
            % TODO : do not check if there is an edge between two vertices or the indexes are the same
            % if vertices(v, 3) ~= vertices(v_i, 3)
                vertex  = [vertices(v, 1), vertices(v, 2)];
                vertex_i  = [vertices(i, 1), vertices(i, 2)];
                angle_mod = mod(atan2(vertex_i(2) - vertex(2), vertex_i(1) - vertex(1)), 2 * pi);
                angle     = atan2(vertex_i(2) - vertex(2), vertex_i(1) - vertex(1));
                A    = [A; angle_mod angle i];
            % end
        end
        A = sortrows(A);
        
end

function [ S ] = intersects_line( v, vertices, E, S )
    % INTERSECTS_LINE Find the edges (E) that intersect with the half line segment
    % starting from the vertex at index v and going to the right.
    %
    %   S: Active list of edges. Array of size N x N, where N is the number of
    %   vertices. Each row represents one vertex, having the distance to the
    %   edge and the index of the vertex in the array of vertices.
    %
    %   v: Index of the initial vertex of the half line segment.
    %
    %   vertices: Array of size N x 3, where N is the number of vertices. Each
    %   row represents one vertex, having the x coordinate, y coordinate and
    %   the object number which represents the polygon to which the vertex
    %   belongs.
    %
    %   E: Array of size M x 2, where M is the number of edges. Each row
    %   represents one edge, having the start and the end vertices index of the
    %   array of vertices.

        [h w] = size(E);
    
        indexed_edge = [];
    
        lines = [];
    
        % traverse the edges
        for i = 1: h
            for j = 1: i 
            
                % check there is an edge between vertices or not
                if E(i, j) == 1 && i ~= v && j ~= v
                    
                    % get the x and y values of the first vertex
                    x_1 = vertices(i, 1);
                    y_1 = vertices(i, 2);
    
                    % get the x and y values of the second vertex
                    x_2 = vertices(j, 1);
                    y_2 = vertices(j, 2);
    
                    lines = [lines; x_1 y_1 x_2 y_2];
                    % line([x_1, x_2], [y_1, y_2], 'Color', 'g');
    
                    x_0 = vertices(v, 1);
                    y_0 = vertices(v, 2);
                    [ distance ,~] = point_segment_distance( [x_0  y_0] , [[x_1 y_1] ; [ x_2 y_2]] );
                    
                    indexed_edge = [indexed_edge; i j distance];
                
                end
            end
        end
    
        current_line = [vertices(v, 1) vertices(v, 2) vertices(v, 1) + cos(0) * 20 vertices(v, 2) + sin(0) * 20];
        % line([vertices(v, 1), vertices(v, 1) + cos(0) * 20], [vertices(v, 2), vertices(v, 2) + sin(0) * 20], 'Color', 'g');
    
        out = lineSegmentIntersect(current_line, lines);
    
        indexes = find(out.intAdjacencyMatrix);
    
        [h w] = size(indexes);
    
        for k = 1: w
            
            i = indexed_edge(indexes(k), 1);
            j = indexed_edge(indexes(k), 2);
            
            S(i, j) = indexed_edge(indexes(k), 3);   
            S(j, i) = indexed_edge(indexes(k), 3);
            
            %{
            % get the x and y values of the first vertex
            x_1 = vertices(i, 1);
            y_1 = vertices(i, 2);
    
            % get the x and y values of the second vertex
            x_2 = vertices(j, 1);
            y_2 = vertices(j, 2);
    
            lines = [lines; [x_1 y_1 x_2 y_2]];
            line([x_1, x_2], [y_1, y_2], 'Color', 'r');
            %}
        end
end





% function [ distance ] = calculate_distance( x_0, y_0, x_1, y_1, x_2, y_2 )
%     % CALCULATE_DISTANCE Calculate the distance between one point to the line IS THIS EVEN CORRECT ??
%     % segment.

%         distance_1 = sqrt((y_1 - y_0)^2 + (x_1 - x_0)^2);
%         distance_2 = sqrt((y_2 - y_0)^2 + (x_2 - x_0)^2);
%         distance = (distance_1 + distance_2) / 2;
% end

function out = lineSegmentIntersect(XY1,XY2)
    %LINESEGMENTINTERSECT Intersections of line segments.
    %   OUT = LINESEGMENTINTERSECT(XY1,XY2) finds the 2D Cartesian Coordinates of
    %   intersection points between the set of line segments given in XY1 and XY2.
    %
    %   XY1 and XY2 are N1x4 and N2x4 matrices. Rows correspond to line segments. 
    %   Each row is of the form [x1 y1 x2 y2] where (x1,y1) is the start point and 
    %   (x2,y2) is the end point of a line segment:
    %
    %                  Line Segment
    %       o--------------------------------o
    %       ^                                ^
    %    (x1,y1)                          (x2,y2)
    %
    %   OUT is a structure with fields:
    %
    %   'intAdjacencyMatrix' : N1xN2 indicator matrix where the entry (i,j) is 1 if
    %       line segments XY1(i,:) and XY2(j,:) intersect.
    %
    %   'intMatrixX' : N1xN2 matrix where the entry (i,j) is the X coordinate of the
    %       intersection point between line segments XY1(i,:) and XY2(j,:).
    %
    %   'intMatrixY' : N1xN2 matrix where the entry (i,j) is the Y coordinate of the
    %       intersection point between line segments XY1(i,:) and XY2(j,:).
    %
    %   'intNormalizedDistance1To2' : N1xN2 matrix where the (i,j) entry is the
    %       normalized distance from the start point of line segment XY1(i,:) to the
    %       intersection point with XY2(j,:).
    %
    %   'intNormalizedDistance2To1' : N1xN2 matrix where the (i,j) entry is the
    %       normalized distance from the start point of line segment XY1(j,:) to the
    %       intersection point with XY2(i,:).
    %
    %   'parAdjacencyMatrix' : N1xN2 indicator matrix where the (i,j) entry is 1 if
    %       line segments XY1(i,:) and XY2(j,:) are parallel.
    %
    %   'coincAdjacencyMatrix' : N1xN2 indicator matrix where the (i,j) entry is 1 
    %       if line segments XY1(i,:) and XY2(j,:) are coincident.
    
    % Version: 1.00, April 03, 2010
    % Version: 1.10, April 10, 2010
    % Author:  U. Murat Erdem
    
    % CHANGELOG:
    %
    % Ver. 1.00: 
    %   -Initial release.
    % 
    % Ver. 1.10:
    %   - Changed the input parameters. Now the function accepts two sets of line
    %   segments. The intersection analysis is done between these sets and not in
    %   the same set.
    %   - Changed and added fields of the output. Now the analysis provides more
    %   information about the intersections and line segments.
    %   - Performance tweaks.
    
    % I opted not to call this 'curve intersect' because it would be misleading
    % unless you accept that curves are pairwise linear constructs.
    % I tried to put emphasis on speed by vectorizing the code as much as possible.
    % There should still be enough room to optimize the code but I left those out
    % for the sake of clarity.
    % The math behind is given in:
    %   http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline2d/
    % If you really are interested in squeezing as much horse power as possible out
    % of this code I would advise to remove the argument checks and tweak the
    % creation of the OUT a little bit.
    
    %%% Argument check.
    %-------------------------------------------------------------------------------
    
    validateattributes(XY1,{'numeric'},{'2d','finite'});
    validateattributes(XY2,{'numeric'},{'2d','finite'});
    
    [n_rows_1,n_cols_1] = size(XY1);
    [n_rows_2,n_cols_2] = size(XY2);
    
    if n_cols_1 ~= 4 || n_cols_2 ~= 4
        error('Arguments must be a Nx4 matrices.');
    end
    
    %%% Prepare matrices for vectorized computation of line intersection points.
    %-------------------------------------------------------------------------------
    X1 = repmat(XY1(:,1),1,n_rows_2);
    X2 = repmat(XY1(:,3),1,n_rows_2);
    Y1 = repmat(XY1(:,2),1,n_rows_2);
    Y2 = repmat(XY1(:,4),1,n_rows_2);
    
    XY2 = XY2';
    
    X3 = repmat(XY2(1,:),n_rows_1,1);
    X4 = repmat(XY2(3,:),n_rows_1,1);
    Y3 = repmat(XY2(2,:),n_rows_1,1);
    Y4 = repmat(XY2(4,:),n_rows_1,1);
    
    X4_X3 = (X4-X3);
    Y1_Y3 = (Y1-Y3);
    Y4_Y3 = (Y4-Y3);
    X1_X3 = (X1-X3);
    X2_X1 = (X2-X1);
    Y2_Y1 = (Y2-Y1);
    
    numerator_a = X4_X3 .* Y1_Y3 - Y4_Y3 .* X1_X3;
    numerator_b = X2_X1 .* Y1_Y3 - Y2_Y1 .* X1_X3;
    denominator = Y4_Y3 .* X2_X1 - X4_X3 .* Y2_Y1;
    
    u_a = numerator_a ./ denominator;
    u_b = numerator_b ./ denominator;
    
    % Find the adjacency matrix A of intersecting lines.
    INT_X = X1+X2_X1.*u_a;
    INT_Y = Y1+Y2_Y1.*u_a;
    INT_B = (u_a >= 0) & (u_a <= 1) & (u_b >= 0) & (u_b <= 1);
    PAR_B = denominator == 0;
    COINC_B = (numerator_a == 0 & numerator_b == 0 & PAR_B);
    
    
    % Arrange output.
    out.intAdjacencyMatrix = INT_B;
    out.intMatrixX = INT_X .* INT_B;
    out.intMatrixY = INT_Y .* INT_B;
    out.intNormalizedDistance1To2 = u_a;
    out.intNormalizedDistance2To1 = u_b;
    out.parAdjacencyMatrix = PAR_B;
    out.coincAdjacencyMatrix= COINC_B;
    
end

function [ draw_line, adjacency_list ] = check_intersection( vertex_1, vertex_2, vertices, edges, S_active_list )
        %check_intersection check whether there is an intersection between two lines or not
        
            draw_line = 0;
        
            % vertex_2'nin edgelerini bul!! belki vertex indexlerini return ettirebiliriz
            adjacency_list = find(edges(vertex_2, :));
        
            if vertices(vertex_1, 3) ~= vertices(vertex_2, 3)
        
                [h w] = size(S_active_list);
                
                weight_lines = [];
        
                % draw edges
                for i = 1: h
        
                    for j = 1: i 
        
                        % check there is an edge between vertices or not
                        if S_active_list(i, j) ~= 0 && i ~= vertex_2 && j ~= vertex_2 && i ~= vertex_1 && j ~= vertex_1
        
                            % get the x and y values of the first vertex
                            x_1 = vertices(i, 1);
                            y_1 = vertices(i, 2);
        
                            % get the x and y values of the second vertex
                            x_2 = vertices(j, 1);
                            y_2 = vertices(j, 2);
        
                            weight_lines = [weight_lines; S_active_list(i, j) i j x_1 y_1 x_2 y_2];
        
                        end
        
                    end
                    
                end
        
                weight_lines = sortrows(weight_lines);
        
                if ~isempty(weight_lines)
                  
                    shortest_line = weight_lines(1, :);
        
                    current_line = [vertices(vertex_1, 1) vertices(vertex_1, 2) vertices(vertex_2, 1) vertices(vertex_2, 2)];
                    % line([vertices(vertex_1, 1), vertices(vertex_2, 1)], [vertices(vertex_1, 2), vertices(vertex_2, 2)], 'Color', 'g');
        
                    out = lineSegmentIntersect(current_line, shortest_line(4: end));
        
                    % isempty(find(out.intAdjacencyMatrix) == 1 ise intersection yok
                    draw_line = isempty(find(out.intAdjacencyMatrix));
        
                else
        
                    draw_line = 1;
        
                end
        
            end
        
end

function [ S_active_list ] = check_active_list( vertex_0, vertex_1, vertices, adjacency_list, S_active_list )
    global closest_vec;

    %check_active_list add or remove the edge to the list S

    if isempty(adjacency_list)
        
        return;
        
    end

    [h w] = size(adjacency_list);
    
    for i = 1: w
        
        vertex_2 = adjacency_list(i);

        if S_active_list(vertex_1, vertex_2) ~= 0 || S_active_list(vertex_2, vertex_1) ~= 0 
            
            S_active_list(vertex_1, vertex_2) = 0;
            
            S_active_list(vertex_2, vertex_1) = 0;
            
        else

            % get the x and y values of the first vertex
            x_1 = vertices(vertex_1, 1);
            y_1 = vertices(vertex_1, 2);

            % get the x and y values of the second vertex
            x_2 = vertices(vertex_2, 1);
            y_2 = vertices(vertex_2, 2);

            x_0 = vertices(vertex_0, 1);
            y_0 = vertices(vertex_0, 2);
            
            [ distance , ang] = point_segment_distance([ x_0 y_0],[ [x_1, y_1]; [x_2 y_2] ]);

            S_active_list(vertex_1, vertex_2) = distance;
            
            S_active_list(vertex_2, vertex_1) = distance;

            
            if distance < closest_vec(5)
                closest_vec = [[ [x_1, y_1], [x_2 y_2] ], distance, ang];
            end

        end
        
    end
end

function draw_graph( vertices, edges )
    %draw_graph draw the whole graph from given vertices and edges

        figure;

        axis([0 12 0 12]);

        hold on;

        [h w] = size(vertices);

        % draw points
        for i = 1: h

            plot(vertices(i, 1), vertices(i, 2), 'bo');
            
            text(vertices(i, 1), vertices(i, 2), num2str(i), 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

        end

        [h w] = size(edges);

        % draw edges
        for i = 1: h
            
            for j = 1: i 
            
                % check there is an edge between vertices or not
                if edges(i, j) == 1
                    
                    % get the x and y values of the first vertex
                    x_1 = vertices(i, 1);
                    y_1 = vertices(i, 2);

                    % get the x and y values of the second vertex
                    x_2 = vertices(j, 1);
                    y_2 = vertices(j, 2);

                    line([x_1, x_2], [y_1, y_2], 'Color', 'b');
                
                end
            
            end

        end

end

function display_edge( edges )
    %display_edge display the given edge

    [h w] = size(edges);
    
    result = [];

    % draw edges
    for i = 1: h
        
        for j = 1: i
            
            if edges(i, j) ~= 0
            
                result = [result; edges(i, j) i j];
                % result = [result; i j];
            
            end
        
        end

    end

    sortrows(result)
    
    size(result)
    
end