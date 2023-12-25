function drawMap()
%DRAWMAP Draw the map that is created by CREATEMAP function
%   This function draws the map. It also draws the start and
%   goal points of the robot.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map;

% Draw the map
% figure(1), clf, hold on
axis equal
axis([map.limits(1) map.limits(2) map.limits(3) map.limits(4)])


for i = 1:length(map.obstacles)
    plot(map.obstacles{i}, 'FaceColor', 'black', 'EdgeColor', 'k')
end

% Customize plot appearance
set(gca, 'Color', 'white') ;
set(gca, 'LineWidth', 1.2) ;
set(gca, 'FontSize', 12) ;
% grid minor;
end

    
    
    