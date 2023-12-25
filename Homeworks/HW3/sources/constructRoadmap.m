function road_map = constructRoadmap(K_nearest)
%CONSTRUCTROADMAP From the nodes, construct the roadmap by connecting the
%   nodes that are within a certain "reach" of each other. The roadmap is 
%   a graph with nodes and edges.
%       - The nodes are the configurations of the robot.
%       - The edges are the paths that the robot can take between the
%       configurations.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global nodes;

roadMap = [];

% FILEPATH: /d:/2023-24/Education/CENG786/Homeworks/HW3/sources/constructRoadmap.m
% BEGIN: ed8c6549bwf9
for i = 1:size(nodes,1)
    distances = sqrt(sum((nodes - nodes(i,:))'.^2)');
    [~, sortedIndices] = sort(distances);
    kNearestIndices = sortedIndices(2:K_nearest+1);
    for j = 1:length(kNearestIndices)
        if  ~checkPath(nodes(i,:), nodes(kNearestIndices(j),:))
            roadMap = [roadMap; i kNearestIndices(j)];
            % nodes(i,:)
        end
    end
end
% END: ed8c6549bwf9
% draw the roadmap with lines
figure(1);
for i = 1:size(roadMap,1)
    plot([nodes(roadMap(i,1),1) nodes(roadMap(i,2),1)], ...
         [nodes(roadMap(i,1),2) nodes(roadMap(i,2),2)], 'b');
end

roadMap
end

