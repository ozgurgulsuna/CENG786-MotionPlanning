function [ E ] = calculate_edges( vertices, N )
    % CALCULATE_EDGES Transforms the initial set of vertices into a data
    % structure that represents the edges of the polygons.
    %
    %   E: Array of size M x 2, where M is the number of edges. Each row
    %   represents one edge, having the start and the end vertices index of the
    %   array of vertices.
    %   vertices: Array of size N x 3, where N is the number of vertices. Each
    %   row represents one vertex, having the x coordinate, y coordinate and
    %   the object number.
    %   N: Number of vertices

        % initialization of the variables
        E =[];
        edge_idx = 1;
        init_vertex_idx = 1;
        for i=2:N

            % number of the current object
            object_nr = vertices(i-1,3);

            if ( vertices(i-1,3) ~= vertices(i,3) )
                % initial vertex index of this object
                E(edge_idx,:) = [i-1 init_vertex_idx];
                edge_idx = edge_idx + 1;
                init_vertex_idx = i;
            end

            % check if the previous vertex is from the same object
            if vertices(i,3) == object_nr

                % add the edge to the array
                E(edge_idx,:) = [i-1 i];
                edge_idx = edge_idx + 1;

            end

        end
        
        % add the last edge
        E(edge_idx,:) = [N init_vertex_idx];

end




            
