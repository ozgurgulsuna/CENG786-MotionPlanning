function road_map = constructRoadmap(K_nearest)
%CONSTRUCTROADMAP From the nodes, construct the roadmap by connecting the
%   nodes that are within a certain "reach" of each other. The reach is
%   defined as the K_nearest neighbors of each node. The distance between
%   two nodes is the Euclidean distance in high dimensional configuration
%   space.
%   The roadmap is a graph with nodes and edges.
%       - The nodes are the configurations of the robot.
%       - The edges are the paths that the robot can take between the
%       configurations.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global nodes;

road_map = [];

for i = 1:size(nodes,1)
    distances = sqrt(sum((nodes - nodes(i,:))'.^2)');
    [~, sortedIndices] = sort(distances);
    kNearestIndices = sortedIndices(2:K_nearest+1);
    for j = 1:length(kNearestIndices)
        if  ~checkPath(nodes(i,:), nodes(kNearestIndices(j),:))
            road_map = [road_map; i kNearestIndices(j)];
            % nodes(i,:)
        end
    end
end

% draw the roadmap with lines
figure(1);
for i = 1:size(road_map,1)
    plot([nodes(road_map(i,1),1) nodes(road_map(i,2),1)], ...
         [nodes(road_map(i,1),2) nodes(road_map(i,2),2)], 'b');
end
% nodes(end,:);
end

