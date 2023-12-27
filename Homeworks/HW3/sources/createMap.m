function createMap()
%CREATEMAP Create a map for the robot to navigate
%   This function creates a 2D polygonal environments map for
%   the robot motion planning problem. For different constraints
%   the map is changed accordingly.
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

global map base_path;

map = struct('limits', [], 'obstacles', [], 'outside', []);

map_limits = [0 150 0 50];

% % Environment 1
% obstacle1 = polyshape([0 0 25 25], [0 25 25 0]);
% obstacle2 = polyshape([75 75 100 100], [75 100 100 75]);
% obstacle3 = polyshape([0 0 25 25], [75 100 100 75]);
% obstacle4 = polyshape([75 75 100 100], [0 25 25 0]);
% obstacle5 = polyshape([35 35 65 65], [35 65 65 35]);

% map.obstacles{1} = obstacle1;
% map.obstacles{2} = obstacle2;
% map.obstacles{3} = obstacle3;
% map.obstacles{4} = obstacle4;
% map.obstacles{1} = obstacle3;   

% map.limits = map_limits;

% map.outside = polyshape([0 0 100 100  NaN -200 -200 300 300], [0 100 100 0 NaN -200 300 300 -200]);


% Environment 2

ground = polyshape([ 0 0 20 20 50 50 60 60 75 90 100 100 ],[-10 0 0 20 20 0 0 10 10 0 0 -10]);
ground_offset = polybuffer(ground, 2);
ground_path = intersect(ground_offset, polyshape([map_limits(1) map_limits(2) map_limits(2) map_limits(1)], [map_limits(3) map_limits(3) map_limits(4) map_limits(4)]));
ground_path = ground_path.Vertices(1:end-2,:)

xl = ground_path(:,1)';
yl = ground_path(:,2)';

c = 'k';    % Black

step = 0.5;    % Affects point quantity
coeff = 0;    % Affects point density

% figure(3)
for n = 1:numel(xl)-1 
    r = norm([xl(n)-xl(n+1), yl(n)-yl(n+1)]);
    m = round(r/step) + 1;
    x = linspace(xl(n), xl(n+1), m) + coeff*randn(1,m);
    y = linspace(yl(n), yl(n+1), m) + coeff*randn(1,m);
    hold on
    % scatter(x,y,'filled','MarkerFaceColor',c);
end

base_path =interparc(100,xl,yl,'linear');

% figure(3)
% scatter(base_path(:,1),base_path(:,2),'filled','MarkerFaceColor',c);



% obstacle1 = polyshape([0 0 25 25], [0 25 25 0]);
% obstacle2 = polyshape([75 75 100 100], [75 100 100 75]);

map.obstacles{1} = ground;

map.limits = map_limits;

map.outside = polyshape([map_limits(1) map_limits(1) map_limits(2) map_limits(2)  NaN -2*map_limits(2) -2*map_limits(2) 3*map_limits(2) 3*map_limits(2)], [map_limits(3) map_limits(4) map_limits(4) map_limits(3) NaN -2*map_limits(4) 3*map_limits(4) 3*map_limits(4) -2*map_limits(4)]);


% Environment 3
% Add code here to define obstacle for environment 3

% Environment 4
% Add code here to define obstacle for environment 4

% Environment 5
% Add code here to define obstacle for environment 5

end



