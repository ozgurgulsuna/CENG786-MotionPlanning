function plotPotential()

global arena_map arena_limits qgoal approx_map;

% field resolution
res = 0.5;
dimension = size(arena_map{1},2);

% get the radius of the bounding sphere

if dimension == 2 % if the start point is 2D
    x_pot = arena_limits(1):res:arena_limits(2);
    y_pot = arena_limits(3):res:arena_limits(4);
    z_pot = zeros(size(x_pot,2),size(y_pot,2));
    for i=1:size(x_pot,2)
        for j=1:size(y_pot,2)
            z_pot(j,i) = potFunction(0,[x_pot(i) ; y_pot(j)], qgoal); 
        end
    end
    figure(2);
    hnd = surf(x_pot,y_pot,z_pot,z_pot);
    shading interp
    set(hnd,'LineStyle','none');
elseif dimension == 3 % if the start point is 3D
    x_pot = arena_limits(1):res:arena_limits(2);
    y_pot = arena_limits(1):res:arena_limits(2);
    z_pot = arena_limits(1):res:arena_limits(2);
    v_pot = zeros(size(x_pot,2),size(y_pot,2),size(z_pot,2));
    for i=1:size(x_pot,2)
        for j=1:size(y_pot,2)
            for k=1:size(z_pot,2)
                v_pot(k,j,i) = potFunction(0,[x_pot(i) ; y_pot(j) ; z_pot(k)], qgoal); 
                % check point in obstacle
                if v_pot(k,j,i) ~= 100
                    for obst_num = 1: length(arena_map)
                        % center point of the obstacle
                        v = (approx_map{obst_num}(1,:)+approx_map{obst_num}(2,:))/2 ;

                        % radius of the obstacle
                        radius = norm(approx_map{obst_num}(1,:) - approx_map{obst_num}(2,:))/2 ;

                        if norm([x_pot(i) y_pot(j) z_pot(k)] - v)  < radius
                            v_pot(k,j,i) = 100;
                        end
                    end
                end
            end
        end
    end
    figure(2);
    % for i=1:size(x_pot,2)
    %     for j=1:size(y_pot,2)
    %         for k=1:size(z_pot,2)
    %             s =scatter3(i,j,k,100*abs(v_pot(k,j,i)/max(max(max(v_pot)))),'filled');
    %             s.AlphaData = abs(v_pot(k,j,i)/max(max(max(v_pot))));
    %             s.MarkerFaceAlpha = 'flat';
    %             s.MarkerFaceColor = 'b';
    %             hold on;
    %         end
    %     end
    % end

    % x_pot';
    % y_pot';
    % z_pot';


    % now we will plot the vector field
    x = repmat(x_pot , size(y_pot,2),1,size(z_pot,2));
    y = repmat(y_pot' , 1,size(x_pot,2),size(z_pot,2));
    z = permute(x, [1 3 2]);

    s = scatter3(x(:),y(:),z(:),[],v_pot(:),'filled','MarkerFaceAlpha',0.5);
    colormap jet
    % s.AlphaData = v_pot/max(max(max(v_pot)));
    % max(max(max(v_pot)))
    % min(min(min(v_pot)))
    % v_pot()
    
    % scatter3(x,y,z,'filled');

    % [x,y,z] = meshgrid(-10:1:10);
    % v = x.^2 + y.^2 + z.^2;
    % quiver3(x,y,z,v);

    % [x, y, z] = meshgrid(-1.5:0.5:1.5);
    % u = x + cos(4*x) + 3;         % x-component of vector field
    % v = sin(4*x) - sin(2*y);      % y-component of vector field
    % w = -z;                       % z-component of vector field

    % % Using an invisible figure because this will choke most video cards
    % % f = figure('Visible', 'off');
    % figure(2);
    % quiver3(x, y, z, u, v, w);


end





end