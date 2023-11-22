function nabla_Uatt = attrGrad( q , w )
%ATTRGRAD Gradient of the attractive potential function
%   This function computes the gradient of the attractive potential
%   function between the two points q and w in an n-dimensions.
%   The potential should be positive and monotonically decreasing,
%   The selected function is the conic potential function.
%   The function and its gradient is given by:
%
%         U_q = 1/2 * zeta * (q - w)^T * (q - w)  and
%         grad(U_q) = zeta * (q - w)
%
%   where q and w are n-dimensional vectors.
%
%   However, the it grows unbounded as q approaches w. Therefore, we
%   need to limit the potential function to a finite value. This is
%   done by changing the potential function to:
%
%         U_q = dgoal_star * zeta *d(q - w) and
%         grad(U_q) = dgoal_star * zeta * (q - w) / d(q - w)
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

zeta = 1 ;  % Attractive potential gain
dgoal_star = 15 ; % Distance at which the potential is truncated

% Compute the distance between the two points
dgoal = norm(q - w) ;

if dgoal > dgoal_star
    nabla_Uatt = dgoal_star*zeta*(q - w)/norm(q - w) ;
else
    nabla_Uatt = zeta*(q - w) ;
end

end