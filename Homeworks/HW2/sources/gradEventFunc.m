function [value,isterminal,direction] = gradEventFunc(t, q, w)

global epsilon
% global qgoal
% qgoal = w;

dimension = length(w);
% find local minima by gradient
gr = gradFunction(t,q,w)<epsilon/2*(ones(dimension,1));


value = [(norm(gradFunction(t,q,w))>epsilon) ...
                       norm(q-w)<epsilon ...
                    gr(1)*gr(2)<epsilon ] ;

isterminal = [0 1 1];   % Stop the integration
direction = [0 0 0];   % any direction


end