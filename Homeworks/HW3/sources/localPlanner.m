function localPlanner(sampleCount)
%LOCALPLANNER samples the configuration space and checks for collisions.
%   The functions aim is to sample the configuration space and check for
%   collisions. The local planner aspect is guarantees that at least two
%   consquetive samples are collision free.
%
%       sampleCount - number of samples to be taken
%       nodes - the nodes of the graph, each node is a configuration
%       
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global nodes q_init q_goal robot map;

% draw the initial configuration
robot = createRobot(q_init,"draw");

% % select a random configuration
% q_rand = [];
% for links = 1 : length(robot.angles)
%     q_rand = [q_rand rand*2*pi];
% end
% q_rand = [rand*map.limits(2) rand*map.limits(4) q_rand];

% draw the random configuration
% createRobot(q_rand,"draw");

% check for collision
nodes = [q_init];
total = 0;
while total < sampleCount
    % select a random configuration
    q_rand = [];
    for links = 1 : length(robot.angles)
        q_rand = [q_rand rand*2*pi];
    end
    q_rand = [rand*map.limits(2) rand*map.limits(4) q_rand];
        % check for collision
        if ~checkCollision(q_rand)
        % if no collision, add the node to the graph
            % if ~checkPath(nodes(end,:),q_rand) % check for path collision in picking the node, makes it unrandom and slower
            nodes = [nodes; q_rand];
            % draw the node
            createRobot(q_rand,"draw");
            total = total + 1;
            % draw the edge
            % hold on;

            % line([nodes(end-1,1) q_rand(1)],[nodes(end-1,2) q_rand(2)],'Color','k','LineWidth',1);
        end
end

% add the goal to the graph
createRobot(q_goal,"draw");
nodes = [nodes; q_goal];
end