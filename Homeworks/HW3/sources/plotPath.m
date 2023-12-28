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
    % N = floor(sqrt(sum((current_configuration - next_configuration).^2))*N); % number of steps for interpolation wrt distance between nodes
    for i = 1:N
        q = (N-i)/N*current_configuration + i/N*next_configuration;
        createRobot(q,"draw");
        pause(0.05);
    end

end

% plot goal configuration
createRobot(nodes(end,:),"draw");

end