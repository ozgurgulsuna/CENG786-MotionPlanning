function [value,isterminal,direction] = potEventFunc(t, q, w)

global epsilon
% global qgoal
% qgoal = w;

dimension = length(w);
% find local minima by gradient
gr = potFunction(t,q,w)<epsilon*(ones(dimension,1));


value = [(norm(potFunction(t,q,w))>epsilon) ...
                       norm(q-w)<epsilon ...
                    gr(1)*gr(2)<epsilon ] ;

isterminal = [0 0 1];   % Stop the integration
direction = [0 0 0];   % any direction


end