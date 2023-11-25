%
% draw_arena()
%  Draws the current layout of the arena.
%
%  Inputs:  None
%  Outputs: None

function draw_arena();

global arena_map arena_limits qstart qgoal dimension approx_map;


if (dimension == 2)
  xmin = arena_limits(1);
  xmax = arena_limits(2);
  ymin = arena_limits(3);
  ymax = arena_limits(4);

  line([xmin xmin xmax xmax xmin], [ymin ymax ymax ymin ymin]);

  for i = 1:length(arena_map);
    obstacle = arena_map{i};
    patch(obstacle(:,1), obstacle(:,2),'black');
  end
  hold on;
  plot(qstart(1), qstart(2), 'o');
  plot(qgoal(1), qgoal(2), 'x');
  hold off;
  axis tight;
  axis square;
  grid on;
elseif (dimension == 3)
  % xmin = arena_limits(1);
  % xmax = arena_limits(2);
  % ymin = arena_limits(3);
  % ymax = arena_limits(4);
  % zmin = arena_limits(5);
  % zmax = arena_limits(6);

  % line([xmin xmin xmax xmax xmin], [ymin ymax ymax ymin ymin], [zmin zmin zmin zmin zmin]);
  % hold on;
  for i = 1:length(arena_map);
    obstacle = arena_map{i};
    scatter3(obstacle(:,1), obstacle(:,2), obstacle(:,3),'black');
    hold on;
    v = (approx_map{i}(1,:)+approx_map{i}(2,:))/2 ;
    r = norm(approx_map{i}(1,:)-approx_map{i}(2,:))/2;
    [x y z] =sphere ;
    surf(r*x+v(1),r*y+v(2),r*z+v(3),'EdgeColor','none','FaceAlpha',0.1,"FaceColor", [0.8 0.1 0.1]);
    set(gca, 'Projection','perspective')
    camproj('perspective')
    % surf(sphere);
  end
  hold on;
  xlim([arena_limits(1) arena_limits(2)]);
  ylim([arena_limits(1) arena_limits(2)]);
  zlim([arena_limits(1) arena_limits(2)]);
  plot3(qstart(1), qstart(2), qstart(3), 'o');
  plot3(qgoal(1), qgoal(2), qgoal(3), 'x');
  hold off;
  % axis tight;
  % axis square;
  grid on;
else
  error('Invalid dimension');

end
