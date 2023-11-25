function [Y] =plotPath(Y)

global arena_map arena_limits qstart qgoal solver;

dimension = size(arena_map{1},2);

if  dimension == 2 
    figure(1);
    clf;
    draw_arena();
    hold on;
    plot(Y(:,1),Y(:,2),'r');
    plot(qstart(1),qstart(2),'bo');
    plot(qgoal(1),qgoal(2),'go');
    hold off;
    axis(arena_limits);
    axis equal;
    xlabel('x');
    ylabel('y');
    title('ODE45');
    legend('path','start','goal','goal');
    drawnow;


% % field resolution
% res = 0.1;
% dimension = size(arena_map{1},2);

% % get the radius of the bounding sphere

% if dimension == 2 % if the start point is 2D
%     x_pot = arena_limits(1):res:arena_limits(2);
%     y_pot = arena_limits(3):res:arena_limits(4);
%     z_pot = zeros(size(y_pot,2),size(x_pot,2));
%     for i=1:size(x_pot,2)
%         for j=1:size(y_pot,2)
%             z_pot(j,i) = potFunction(0,[x_pot(i) ; y_pot(j)], qgoal); 
%         end
%     end
% end



% figure(2);
% hnd = surf(x_pot,y_pot,z_pot,z_pot);
% shading interp
% set(hnd,'LineStyle','none');
end