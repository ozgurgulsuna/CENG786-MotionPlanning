function [value,isterminal,direction] = potEventFunc2(t, q, w)

global epsilon
% global qgoal
% qgoal = w;

dimension = length(w);
% find local minima by gradient
gr = attr_repl(t,q,w)<epsilon*(ones(dimension,1));


value = [ norm(q-w)<epsilon  ] ;   

isterminal = [1];   % Stop the integration
direction = [0];   % any direction


end