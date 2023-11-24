function plotPotential()

global arena_map arena_limits qgoal;

% field resolution
res = 0.05;
dimension = size(arena_map{1},2);

% get the radius of the bounding sphere

if dimension == 2 % if the start point is 2D
    x_pot = arena_limits(1):res:arena_limits(2);
    y_pot = arena_limits(3):res:arena_limits(4);
    z_pot = zeros(size(y_pot,2),size(x_pot,2));
    for m=1:length(arena_map)
        for i=1:size(x_pot,2)
            for j=1:size(y_pot,2)
                if inpolygon(x_pot(i),y_pot(j),arena_map{m}(:,1),arena_map{m}(:,2))
                    z_pot(j,i) = 100; 
                else
                    z_pot(j,i) = potFunction(0,[x_pot(i) ; y_pot(j)], qgoal); 
                end            
            end
        end
    end
end


figure(2);
hnd = surf(x_pot,y_pot,z_pot,z_pot);
set(hnd,'LineStyle','none');
end