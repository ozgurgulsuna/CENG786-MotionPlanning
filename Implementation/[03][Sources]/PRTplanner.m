%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% POLYGONAL RANDOM TREE SEARCH on IRREGULAR TERRAIN
%  ───────────────────────────────
%  future work: make it an RRT* search
%  ───────────────────────────────
function PRTplanner(p_init, s_goal, size)

    % global variables
    global terrain;
    global mesh;
    global tree;

    % initialize variables
    goal_reached = false;
    mixing_factor = 0.1;

    % initialize terrain
    bounding_box = max(mesh.Vertices) - min(mesh.Vertices);

    % initialize tree structure
    tree = struct('polygon', [], 'parent', []);
    tree(1).polygon = p_init;
    tree(1).parent = 0;

    while ~goal_reached
    % for i = 1:200
        % generate random point
        s_rand = [rand*bounding_box(1), rand*bounding_box(2), rand*bounding_box(3)];
            if rand >= mixing_factor
                s_obj = s_rand;
            else
                s_obj = s_goal;
            end
        
        [temp_polygon , parent_polygon] = findNearest(s_obj,tree,size);

        % check if the polygon is slanted
        temp_polygon_normal = meshNormal3d(temp_polygon);
        vertical_direction = [0 0 -1];
        if atan2(norm(cross(temp_polygon_normal,vertical_direction)),dot(temp_polygon_normal,vertical_direction))/pi < 1
        % if atan2(norm(cross(temp_polygon_normal,vertical_direction)),dot(temp_polygon_normal,vertical_direction))/pi < 0.085
            tree(end+1).polygon = temp_polygon;
            tree(end).parent = parent_polygon;
        else
            % fprintf('slanted polygon, not added to the tree\n');
        end



        if length(tree) > 500
            error('Took so many iterations, something is wrong')
            break;
        end

        % check if the goal is reached
        if norm(s_goal - temp_polygon(1,:)) < 5
            temp_polygon(1,:)
            s_goal
            norm(s_goal - temp_polygon(1,:))
            goal_reached = true;
            fprintf('goal reached\n');
        end
    end
    
    % % backtracking
    solved_tree = [];
    solved_tree(1).polygon = tree(end).polygon;
    while parent_polygon ~= 0
        solved_tree(end+1).polygon = tree(parent_polygon).polygon;
        parent_polygon = tree(parent_polygon).parent;
    end
    solved_tree(end+1).polygon = p_init;
    solved_tree = flip(solved_tree);

    % solved_tree


    % fprintf('done\n');

    % plot all polygons
    figure;
    % set(gcf, 'Renderer', 'painters');
    % set the figure size
    set(gcf, 'Position',  [100, 50, 1200, 800])
    hold on;
    % scatter3(mesh.Vertices(:,1), mesh.Vertices(:,2), mesh.Vertices(:,3), 1, 'k');
    scatter3(terrain(:,1), terrain(:,2), terrain(:,3), 1, 'k' );
    xlim([0 100]);
    ylim([0 100]);
    zlim([0 20]);

    % view(3);
    % set the camera angle
    view(35 ,-50);

    % set camera position
    campos([-280,-350,350]);

    % set camera target
    camtarget([36,50,2.5]);

    % set camera up vector
    camup([0,0,1]);

    % set camera view angle
    camva(11);
    
    axis equal;

    pause(1);

    for i = 1: length(tree)
        polygon = tree(i).polygon;
        for j = 1:3
            plot3([polygon(j,1), polygon(mod(j,3)+1,1)], [polygon(j,2), polygon(mod(j,3)+1,2)], [polygon(j,3), polygon(mod(j,3)+1,3)], 'b');
            pause(0.01);
        end
        hold on;
    end

    % plot the solved path
    for i = 1: length(solved_tree)
        polygon = solved_tree(i).polygon;
        for j = 1:3
            plot3([polygon(j,1), polygon(mod(j,3)+1,1)], [polygon(j,2), polygon(mod(j,3)+1,2)], [polygon(j,3), polygon(mod(j,3)+1,3)], 'r');
            pause(0.01);
        end
        hold on;
    end





end

function [ new_polygon , parent_polygon ] = findNearest(s_obj, tree, size)
    % given the tree structure and an object point, find the nearest foot alternative 
    % the algorithm searches every polygon in the mesh and finds the nearest foot
    % future work: only use the outeredges of the polygons in the tree
    
    % global variables
    global terrain;
    global mesh;

    % initialize variables
    max_distance = 0;
    scaling_factor = 0.1;

    % edge selection matrix
    edge_selection = [2 3; 3 1; 1 2];
    min_distance = inf;

    for i = 1: length(tree)
        polygon = tree(i).polygon;
        polygon_normal = meshNormal3d(polygon);

        if i == 1
            edge_start = 1;
        else
            edge_start = 2;
        end
        for j = edge_start:3
            foot_direction = cross(polygon_normal, (polygon(edge_selection(j,1),:)-polygon(edge_selection(j,2),:))/(norm(polygon(edge_selection(j,1),:)-polygon(edge_selection(j,2),:))));
            scale = size + scaling_factor*(1-(norm(polygon(edge_selection(j,1),:)-polygon(edge_selection(j,2),:))));
            foot_origin = (polygon(edge_selection(j,1),:)/2 + polygon(edge_selection(j,2),:)/2) +  sqrt(3)/2*(norm(polygon(edge_selection(j,1),:)-polygon(edge_selection(j,2),:))) * foot_direction*scale;
            foot_line = [ foot_origin , polygon_normal ];
            % here due to our planar assumption, the robot member length increase TO DO : SOLVE THIS
            [foot b c] = intersectLineMesh3d(foot_line, mesh.Vertices, mesh.Faces);
            
            % throw an error if the foot is not found
            if isempty(foot)
                % error('No intersection: foot is not found');
                continue;
            end

            obj_distance = distanceMetric(s_obj, foot, polygon_normal);
            if obj_distance < min_distance
                min_distance = obj_distance;
                % the order of the polygon is important, reversed since it is the new polygon
                new_polygon = [foot; polygon(edge_selection(j,2),:); polygon(edge_selection(j,1),:)];
                parent_polygon = i;
            end

        end

    end

end

function direction = meshNormal3d(polygon)
    % given a polygon, calculate the normal of the polygon
    % future work: use the normal of the polygon from the mesh
    % https://www.maths.usyd.edu.au/u/MOW/vectors/vectors-11/v-11-7.html

    V1 = polygon(1,:);
    V2 = polygon(2,:);
    V3 = polygon(3,:);

    direction = cross(V1-V3,V2-V3);

    area=norm(direction)/2;

    direction=direction/(2*area); %normalized equivalent
end



function distance = distanceMetric(goal, start, normal)
    % given an object point and a foot point, calculate the distance between them

    % THE NORMAL IS WRONG, IT SHOULD BE THE NORMAL OF THE NEW POLYGON
    
    % distance = norm(goal - start);
    u = normal;
    v = [0 0 1];
    distance = norm(goal - start) + 0*(atan2(norm(cross(u,v)),dot(u,v)));


end

function [points pos faceInds] = intersectLineMesh3d(line, vertices, faces)
    %INTERSECTLINEMESH3D Intersection points of a 3D line with a mesh
    %
    %   INTERS = intersectLineMesh3d(LINE, VERTICES, FACES)
    %   Compute the intersection points between a 3D line and a 3D mesh defined
    %   by vertices and faces.
    %
    %   [INTERS POS INDS] = intersectLineMesh3d(LINE, VERTICES, FACES)
    %   Also returns the position of each intersection point on the input line,
    %   and the index of the intersected faces.
    %   If POS > 0, the point is also on the ray corresponding to the line. 
    %   
    %   Example
    %   intersectLineMesh3d
    %
    %   See also
    %   meshes3d, triangulateFaces
    %
    % ------
    % Author: David Legland
    % e-mail: david.legland@grignon.inra.fr
    % Created: 2011-12-20,    using Matlab 7.9.0.529 (R2009b)
    % Copyright 2011 INRA - Cepia Software Platform.
    
    
    tol = 1e-12;
    
    % ensure the mesh has triangular faces
    tri2Face = [];
    if iscell(faces) || size(faces, 2) ~= 3
        [faces tri2Face] = triangulateFaces(faces);
    end
    
    % find triangle edge vectors
    t0  = vertices(faces(:,1), :);
    u   = vertices(faces(:,2), :) - t0;
    v   = vertices(faces(:,3), :) - t0;
    
    % triangle normal
    n   = normalizeVector3d(crossProduct3d(u, v));
    
    % direction vector of line
    dir = line(4:6);
    
    % vector between triangle origin and line origin
    w0 = bsxfun(@minus, line(1:3), t0);
    
    a = -dot(n, w0, 2);
    b = dot(n, repmat(dir, size(n, 1), 1), 2);
    
    valid = abs(b) > tol & vectorNorm3d(n) > tol;
    
    % compute intersection point of line with supporting plane
    % If pos < 0: point before ray
    % IF pos > |dir|: point after edge
    pos = a ./ b;
    
    % coordinates of intersection point
    points = bsxfun(@plus, line(1:3), bsxfun(@times, pos, dir));
    
    
    %% test if intersection point is inside triangle
    
    % normalize direction vectors of triangle edges
    uu  = dot(u, u, 2);
    uv  = dot(u, v, 2);
    vv  = dot(v, v, 2);
    
    % coordinates of vector v in triangle basis
    w   = points - t0;
    wu  = dot(w, u, 2);
    wv  = dot(w, v, 2);
    
    % normalization constant
    D = uv.^2 - uu .* vv;
    
    % test first coordinate
    s = (uv .* wv - vv .* wu) ./ D;
    ind1 = s < 0.0 | s > 1.0;
    points(ind1, :) = NaN;
    pos(ind1) = NaN;
    
    % test second coordinate, and third triangle edge
    t = (uv .* wu - uu .* wv) ./ D;
    ind2 = t < 0.0 | (s + t) > 1.0;
    points(ind2, :) = NaN;
    pos(ind2) = NaN;
    
    % keep only interesting points
    inds = ~ind1 & ~ind2 & valid;
    points = points(inds, :);
    
    pos = pos(inds);
    faceInds = find(inds);
    
    % convert to face indices of original mesh
    if ~isempty(tri2Face)
        faceInds = tri2Face(faceInds);
    end

end

function c = crossProduct3d(a,b)
    %CROSSPRODUCT3D Vector cross product faster than inbuilt MATLAB cross.
    %
    %   C = crossProduct3d(A, B) 
    %   returns the cross product of the two 3D vectors A and B, that is: 
    %       C = A x B
    %   A and B must be N-by-3 element vectors. If either A or B is a 1-by-3
    %   row vector, the result C will have the size of the other input and will
    %   be the  concatenation of each row's cross product. 
    %
    %   Example
    %     v1 = [2 0 0];
    %     v2 = [0 3 0];
    %     crossProduct3d(v1, v2)
    %     ans =
    %         0   0   6
    %
    %
    %   Class support for inputs A,B:
    %      float: double, single
    %
    %   See also DOT.
    %   Sven Holcombe
    % HISTORY
    % 2017-11-24 rename from vectorCross3d to crossProduct3d
    % size of inputs
    sizeA = size(a);
    sizeB = size(b);
    % Initialise c to the size of a or b, whichever has more dimensions. If
    % they have the same dimensions, initialise to the larger of the two
    switch sign(numel(sizeA) - numel(sizeB))
        case 1
            c = zeros(sizeA);
        case -1
            c = zeros(sizeB);
        otherwise
            c = zeros(max(sizeA, sizeB));
    end
    c(:) = bsxfun(@times, a(:,[2 3 1],:), b(:,[3 1 2],:)) - ...
           bsxfun(@times, b(:,[2 3 1],:), a(:,[3 1 2],:));
    
end

function n = vectorNorm3d(v)
    %VECTORNORM3D Norm of a 3D vector or of set of 3D vectors.
    %
    %   N = vectorNorm3d(V);
    %   Returns the norm of vector V.
    %
    %   When V is a N-by-3 array, compute norm for each vector of the array.
    %   Vector are given as rows. Result is then a N-by-1 array.
    %
    %   NOTE: compute only euclidean norm.
    %
    %   See Also
    %   vectors3d, normalizeVector3d, vectorAngle3d, hypot3
    %
    %   ---------
    %   author : David Legland 
    %   INRA - TPV URPOI - BIA IMASTE
    %   created the 21/02/2005.
    %   HISTORY
    %   19/06/2009 rename as vectorNorm3d
    n = sqrt(sum(v.*v, 2));
end

function vn = normalizeVector3d(v)
    %NORMALIZEVECTOR3D Normalize a 3D vector to have norm equal to 1.
    %
    %   V2 = normalizeVector3d(V);
    %   Returns the normalization of vector V, such that ||V|| = 1. Vector V is
    %   given as a row vector.
    %
    %   If V is a N-by-3 array, normalization is performed for each row of the
    %   input array.
    %
    %   See also:
    %   vectors3d, vectorNorm3d
    %
    %   ---------
    %   author : David Legland 
    %   INRA - TPV URPOI - BIA IMASTE
    %   created the 29/11/2004.
    %
    % HISTORY
    % 2005-11-30 correct a bug
    % 2009-06-19 rename as normalizeVector3d
    % 2010-11-16 use bsxfun (Thanks to Sven Holcombe)
    vn   = bsxfun(@rdivide, v, sqrt(sum(v.^2, 2)));
end    