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
  plot(qstart(1), qstart(2), 'o','MarkerSize',10); %'MarkerFaceColor','b'
  plot(qgoal(1), qgoal(2), 'x','MarkerSize',10); %'MarkerFaceColor','b'
  hold off;
  axis tight;
  axis square;
  grid on;
elseif (dimension >= 3)
  for i = 1:length(arena_map);
    obstacle = arena_map{i};
    scatter3(obstacle(:,1), obstacle(:,2), obstacle(:,3),'o','filled','MarkerFaceColor',hsv2rgb([i/length(arena_map) 0.7 0.8]));
    hold on;
    v = (approx_map{i}(1,:)+approx_map{i}(2,:))/2 ;
    r = norm(approx_map{i}(1,:)-approx_map{i}(2,:))/2;
    [x y z] =sphere ;
    surf(r*x+v(1),r*y+v(2),r*z+v(3),'EdgeColor','none','FaceAlpha',0.1,"FaceColor", hsv2rgb([i/length(arena_map) 0.7 0.8]));
    set(gca, 'Projection','perspective')
    camproj('perspective')
  end
  hold on;
  xlim([arena_limits(1) arena_limits(2)]);
  ylim([arena_limits(1) arena_limits(2)]);
  zlim([arena_limits(1) arena_limits(2)]);
  plot3(qstart(1), qstart(2), qstart(3), 'o');
  plot3(qgoal(1), qgoal(2), qgoal(3), 'x');
  hold off;
  grid on;
else
  error('Invalid dimension');

end
