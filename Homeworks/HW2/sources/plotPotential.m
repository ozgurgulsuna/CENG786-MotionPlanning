function plotPotential()

global arena_map arena_limits qgoal approx_map;

% field resolution
res = 0.3;
dimension = size(arena_map{1},2);

% get the radius of the bounding sphere

if dimension == 2 % if the start point is 2D
    res = 0.1;
    x_pot = arena_limits(1):res:arena_limits(2);
    y_pot = arena_limits(1):res:arena_limits(2);
    z_pot = zeros(size(x_pot,2),size(y_pot,2));
    for i=1:size(x_pot,2)
        for j=1:size(y_pot,2)
            z_pot(j,i) = potFunction(0,[x_pot(i) ; y_pot(j)], qgoal); 
            for m = 1: length(arena_map)
                if z_pot(j,i) ~= 100
                    if inpolygon(x_pot(i),y_pot(j), arena_map{m}(:,1), arena_map{m}(:,2))
                        z_pot(j,i) = 100;
                    end
                end
            end
        end
    end
    figure(2);

    X = repmat(x_pot(:),1,length(y_pot(:)));
    Y = repmat(y_pot(:)',length(x_pot(:)),1);

    Normals = [X(:)  Y(:)  z_pot(:)]; %on a sphere the points reflect the normal direction
    mappedRGB = Sphere2RGBCube(Normals);
    size(mappedRGB);
    image = reshape(mappedRGB, [length(x_pot(:)), length(y_pot(:)), 3]);
    image = imresize(image, 40, 'cubic');

    hold on
    % scatter3(Y(:),X(:),z_pot(:),50,mappedRGB,'.');
    surf(double(Y),double(X),z_pot,'FaceColor','texturemap','CData',image,'EdgeColor','none');
    % shading interp
    % colormap hsv
elseif dimension == 3 % if the start point is 3D
    x_pot = arena_limits(1):res:arena_limits(2);
    y_pot = arena_limits(1):res:arena_limits(2);
    z_pot = arena_limits(1):res:arena_limits(2);
    v_pot = zeros(size(x_pot,2),size(y_pot,2),size(z_pot,2));
    for i=1:size(x_pot,2)
        for j=1:size(y_pot,2)
            for k=1:size(z_pot,2)
                v_pot(k,j,i) = -potFunction(0,[x_pot(i) ; y_pot(j) ; z_pot(k)], qgoal); 
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
    % now we will plot the vector field
    y = repmat(x_pot , size(y_pot,2),1,size(z_pot,2));
    z = repmat(y_pot' , 1,size(x_pot,2),size(z_pot,2));
    x = permute(z, [3 2 1]);
    
    s = scatter3(x(:),y(:),z(:),abs(v_pot(:)+0.01),v_pot(:),'o','MarkerEdgeAlpha',0.2);
    colormap hsv

end


end