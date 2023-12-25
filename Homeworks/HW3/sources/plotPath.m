function plotPath(path)
%PLOTPATH Summary of this function goes here

global nodes;

N = 20; % number of steps for interpolation

% open new figure window
% figure;
% hold on;
drawMap(2);

% plot start configuration
createRobot(nodes(1,:),"draw");

for i = 1:length(path)-1
    current_configuration = nodes(path(i),:);
    next_configuration = nodes(path(i+1),:);
    for i = 1:N
        q = (N-i)/N*current_configuration + i/N*next_configuration;
        createRobot(q,"draw");
    end

end

% plot goal configuration
createRobot(nodes(end,:),"draw");

end