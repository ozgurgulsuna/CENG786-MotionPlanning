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

createRobot(q_rand,"draw");

