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

% select a random configuration
q_rand = [];
for links = 1 : length(robot.angles)
    q_rand = [q_rand rand*2*pi];
end
q_rand = [rand*map.limits(2) rand*map.limits(4) q_rand];

% draw the random configuration
% createRobot(q_rand,"draw");

% check for collision
if ~checkCollision(q_rand)
    % if no collision, add the node to the graph
    nodes = [nodes; q_rand];
    % draw the node
    createRobot(q_rand,"draw");
    % draw the edge
    line([q_init(1) q_rand(1)],[q_init(2) q_rand(2)],'Color','k','LineWidth',1);
    % check if the goal is reached
    if norm(q_rand(1:2)-q_goal(1:2)) < 0.1
        % if goal is reached, stop the simulation
        disp("Goal is reached!");
        return;
    end
end
%     % if no collision, add the node to the graph
%     nodes = [nodes; q_rand];
%     % draw the node
%     createRobot(q_rand,"draw");
%     % draw the edge
%     line([q_init(1) q_rand(1)],[q_init(2) q_rand(2)],'Color','k','LineWidth',1);
%     % check if the goal is reached
%     if norm(q_rand(1:2)-q_goal(1:2)) < 0.1
%         % if goal is reached, stop the simulation
%         disp("Goal is reached!");
%         return;
%     end
% end
