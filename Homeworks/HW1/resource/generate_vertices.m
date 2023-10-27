function [ vertices ] = generate_vertices( obstacle )
  %GENERATE_VERTICES Generate the vertices of the environment
  %   - obstacle: array of size N x 2, where each row [x, y] represents
  %     the x coordinate and y coordinate of the obstacle. N is the number
  %     of obstacles, which is not pre-defined, given that it depends on
  %     the problem.
  %   - vertices: array of size M x 3, where each row [x, y, n] represents
  %     the x coordinate, y coordinate and object number. M is the number of
  %     vertices, which is not pre-defined, given that it depends on the
  %     problem.
  
  % number of obstacles
  N = size(obstacle,2);
  
  % intitialise the output vector
  vertices = [];
  
  % index of the next vertex to be inserted
  vertices_idx = 1;
  
  % iterate through all the obstacles
  for i=1:N
      
      % number of vertices of the current obstacle
      n = size(obstacle{i},1);
      
      % iterate through all the vertices of the current obstacle
      for j=1:n
          
          % add the vertex to the output vector
          vertices(vertices_idx,:) = [obstacle{i}(j,:), i];
          
          % increment the index of the next vertex to be inserted
          vertices_idx = vertices_idx + 1;
      end
  end
end
