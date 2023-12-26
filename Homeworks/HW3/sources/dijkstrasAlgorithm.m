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
    %   euclidean distance, less weight on the angular distance.
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
    A = full(sparse(s,t,G.Edges.Weight,nn,nn))
    
    % then compute the shortest path
    path = shortestpath(G,1,size(nodes,1))
    
    end