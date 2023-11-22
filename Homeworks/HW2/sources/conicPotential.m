function [U_q] = conicPotential( q , w )
%CONICPOTENTIAL conic potential function
%   This function computes the conic potential function between the
%   two points q and w in an n-dimensional workspace. The potential
%   function is given by:
%         U_q = 1/2 * (q - w)^T * (q - w)
%   where q and w are n-dimensional vectors.
%
%   - Input:  q = [q_1, q_2, ..., q_n]  n-dimensional coordinate vector
%             w = [w_1, w_2, ..., w_n]  n-dimensional coordinate vector
%
%   - Output: U_q = scalar conic potential value
%
%   Ozgur Gulsuna, METU
%   CENG786 Robot Motion Planning and Control, Fall 2023

U_q = 0.5 * (q - w)' * (q - w);

end

