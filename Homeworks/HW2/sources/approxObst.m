function approx_map = approxObst(arena_map)
%APPROXOBST Summary of this function goes here
% n dimensional sphere approximation of the obstacles

% here we will approximate the obstacles by an n-dimensional sphere (why not ellipsoid?)
% we will use the diameter of the approximated obstacle as the distance between the most
% distant points of the obstacle. The center of the sphere will be the middle point of
% of the line segment connecting the two most distant points of the obstacle.

% we will use the following algorithm:

for m = 1: length(arena_map)
    diameters{m} = 0;
end

for m = 1: length(arena_map)
    approx_map{m} = [0 ; 0];
end

for m = 1: length(arena_map)
    v = 1: length(arena_map{m});
    C = nchoosek(v,2);
    for i = 1: length(C)
        norm_dist(i) = norm(arena_map{m}(C(i,1),:) - arena_map{m}(C(i,2),:));
        if norm_dist(i) > diameters{m}
            diameters{m} = norm_dist(i);
            approx_map{m} = [ [arena_map{m}(C(i,1),:)] ; [arena_map{m}(C(i,2),:)] ];
        end
    end
end


end