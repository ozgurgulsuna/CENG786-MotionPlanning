%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% POLYGONAL RANDOM TREE SEARCH on IRREGULAR TERRAIN
%  ───────────────────────────────
%  future work: make it an RRT* search
%  ───────────────────────────────
function PRTplanner(p_init, s_goal)

    % global variables
    global terrain;
    global mesh;

    % initialize variables
    goal_reached = false;
    mixing_factor = 0.2;

    % initialize terrain
    bounding_box = max(mesh.Vertices) - min(mesh.Vertices);

    % initialize tree
    tree = [p_init]; % tree = [ polygon , edge ]

    while ~goal_reached
        % generate random point
        s_rand = [rand*bounding_box(1), rand*bounding_box(2), rand*bounding_box(3)];
            if rand >= mixing_factor
                s_obj = s_rand;
            else
                s_obj = s_goal;
            end
        
        temp_foot = findNearest(s_obj);


        % % find nearest node
        % [p_near, p_near_idx] = find_nearest_node(p_rand);
        % % find new point
        % p_new = find_new_point(p_near, p_rand);
        % % check if new point is valid
        % if is_valid_point(p_new)
        %     % add new point to tree
        %     tree = add_new_point(p_new, p_near_idx);
        %     % check if goal is reached
        %     goal_reached = is_goal_reached(p_new);
        % end
    end


end

function foot = findNearest(s_obj, tree)
    % given the tree structure and an object point, find the nearest foot alternative 
    % the algorithm searches every polygon in the mesh and finds the nearest foot
    % future work: only use the outeredges of the polygons in the tree
    
    % global variables
    global terrain;
    global mesh;

    % initialize variables

    % for i = 1: length(tree)
    %     % get the polygon
    %     polygon = tree(i).polygon;
    %     % get the edges
    %     edges = tree(i).edges;
    %     % get the vertices
    %     vertices = tree(i).vertices;
    %     % get the normals
    %     normals = tree(i).normals;
    %     % get the foot
    %     foot = findFoot(s_obj, polygon, edges, vertices, normals);
    % end


    poly = [ [-0.5 -sqrt(3)/6 0] ; [0.5 -sqrt(3)/6 0] ; [0 sqrt(3)/3 0] ]

    poly = poly + [ [45 45 45] ; [45 45 45] ; [45 45 45] ]

    poly_normal = polyNormal(poly)

    foot_direction = cross(poly_normal, (poly(1,:)-poly(2,:))/(norm(poly(1,:)-poly(2,:))))

    foot_origin = (poly(1,:)/2 + poly(2,:)/2) +  sqrt(3)/2*(norm(poly(1,:)-poly(2,:))) * foot_direction

    foot_line = [ foot_origin , poly_normal ]

    % here due to our planar assumption, the robot member length increase TO DO : SOLVE THIS

    % figure
    % scatter3(poly(:,1),poly(:,2),poly(:,3))
    % hold on
    % scatter3(foot_origin(1),foot_origin(2),foot_origin(3))
    % hold on
    % quiver3(foot_origin(1),foot_origin(2),foot_origin(3),poly_normal(1),poly_normal(2),poly_normal(3))


    % xlim([48 52])
    % ylim([48 52])
    % zlim([48 52])

    [a b c] = intersectLineMesh3d(foot_line, mesh.Vertices, mesh.Faces)

    figure
    scatter3(mesh.Vertices(:,1),mesh.Vertices(:,2),mesh.Vertices(:,3),'k.')
    hold on
    scatter3(a(:,1),a(:,2),a(:,3))

    zlim([0 10])

end

function direction = polyNormal(polygon)
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



function distance = distanceMetric(s_obj, foot)
    % given an object point and a foot point, calculate the distance between them
    % future work: use the distance metric from the paper
    distance = norm(s_obj - foot);

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