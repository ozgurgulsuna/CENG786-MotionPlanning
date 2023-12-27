function path = dijkstrasAlgorithm(road_map)
%SHORTESTPATH using Dijkstra's algorithm to find the shortest path
%   between two nodes in a graph. The graph is a roadmap in the
%   configuration space of a robot. The nodes are configurations
%   of the robot. The edges are collision-free paths between
%   configurations. The weight of each edge is the distance between
%   the two configurations. The distance is high dimensional and
%   is computed using a distance metric.
%
%   TODO: add weight to the distance metric, more weight on the
%   cartesian distance, less weight on the angular distance.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global nodes;

% first convert the roadmap to a graph
s = road_map(:,1)';
t = road_map(:,2)';
w = distanceMetric(nodes(s,:),nodes(t,:));
G = graph(s,t,w);

nn = numnodes(G);
A = full(sparse(s,t,G.Edges.Weight,nn,nn));

A=A+1./(A~=0)-1;

visited(1:nn) = 0;
distance(1:nn) = inf;    % it stores the shortest distance between each node and the source node;
parent(1:nn) = 0;
distance(1) = 0;

for i = 1:(nn-1)
    temp = [];
    for h = 1:nn
         if visited(h) == 0   % in the tree;
             temp=[temp distance(h)];
         else
             temp=[temp inf];
         end
     end
     [t, u] = min(temp);    % it starts from node with the shortest distance to the source;
     visited(u) = 1;       % mark it as visited;
     for v = 1:nn           % for each neighbors of node u;
         if ( ( A(u, v) + distance(u)) < distance(v) )
             distance(v) = distance(u) + A(u, v);   % update the shortest distance when a shorter path is found;
             parent(v) = u;                                     % update its parent;
         end             
     end
end

path = [];
if parent(nn) ~= 0   % if there is a path!
    t = nn;
    path = [nn];
    while t ~= 1
        p = parent(t);
        path = [p path];
        t = p;      
    end
end

if isempty(path)
    disp('No path found!');
end
path
path = shortestpath(G,1,nn)
end
