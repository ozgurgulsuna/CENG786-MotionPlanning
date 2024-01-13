% syms x y z

% F = [ x*y*z, x*y, x*z, y*z, x, y, z, 1];

% x = [x y z];

% J = jacobian(F,x);

% connectiivty matrix generation

C = zeros(M,N);

for i = 1:M
    C(i,trussConnectivity(i,1)) = 1;
    C(i,trussConnectivity(i,2)) = -1;
end

% create a node matrix from x y z
p_x = trussNodes(:,1);
p_y = trussNodes(:,2);
p_z = trussNodes(:,3);

p = [p_x p_y p_z];

C*p_x
C*p_y
C*p_z

d = [C*p_x C*p_y C*p_z];

for i = 1:M
    Q(i) = norm(d(i,:));
end



%%%%%%%%%%%%%%

L = zeros(M,1);
for i = 1:M
    L(i) = norm(trussNodes(trussConnectivity(i,1),:) - trussNodes(trussConnectivity(i,2),:));
end
