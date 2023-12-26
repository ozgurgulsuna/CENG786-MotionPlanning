function distance = distanceMetric(nodes1, nodes2)
%DISTANCEMETRIC Summary of this function goes here
%   nodes1 = [x1 y1 theta1 theta2; x2 y2 theta1 theta2; ...]
%   nodes2 = [z1 z1 alpha1 alpha2; z2 z2 alpha1 alpha2; ...]

% global nodes;
weight = [1 1 10 10];
nodes1 = nodes1.*weight;
nodes2 = nodes2.*weight;
distance = sqrt((sum((nodes1'-nodes2').^2)));

% distance = sqrt(sum((nodes(s,:)-nodes(t,:)).^2'))

end
