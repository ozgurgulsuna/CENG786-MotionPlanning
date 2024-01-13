%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% CREATE ROBOT
% The robot created can have different number of nodes and members. Selected topology is octahedron.
%
clc
clear



N = 6; % Number of nodes
M = 12; % Number of members

% Node Coordinates
% Nodes are ordered as starting from  the bottom polygon, going in clockwise
% direction and then going to the top polygon in a spiral way.
trussNodes  =  [-1 0 0; ...  % Node 1 x,y,z
                0 0 -1; ...  % Node 2 x,y,z
                0 -1 0; ...  % Node 3
                0 0 1; ...  % ...
                0 1 0; ...
                1 0 0; ...
                ];


% Connectivity Matrix
% Members are ordered as starting from the bottom polygon, if are in same polygon, going in clockwise
% direction and then going up, rotating clockwise again if in the same plane.

trussConnectivity = [1 2; ... % Member_1 -> node-1 node-2
                     2 3; ... % Member_2 -> node-2 node-3
                     3 1; ... % Member_3 -> node-3 node-1
                     1 4; ... % Member_4 -> node-1 node-4
                     1 5; ... % Member_5 -> node-1 node-5
                     2 5; ... % Member_6 -> node-2 node-5
                     2 6; ... % Member_7 -> node-2 node-6
                     3 6; ... % Member_8 -> node-3 node-6
                     3 4; ... % Member_9 -> node-3 node-4
                     4 5; ... % Member_10 -> node-4 node-5
                     5 6; ... % Member_11 -> node-5 node-6
                     6 4; ... % Member_12 -> node-6 node-4
                     ];



% Member lengths
trussMembers = [1 ; ... % Member_1 length
                1 ; ... % Member_2 length
                1 ; ... % Member_3 length
                1 ; ... % Member_4 length
                1 ; ... % Member_5 length
                1 ; ... % Member_6 length
                1 ; ... % Member_7 length
                1 ; ... % Member_8 length
                1 ; ... % Member_9 length
                1 ; ... % Member_10 length
                1 ; ... % Member_11 length
                1 ; ... % Member_12 length
                ];

robotTopology = struct('nodes', [trussNodes], ...
                       'connections', [trussConnectivity], ...
                       'lengths', [trussMembers], ...
                       'numNodes', N, ...
                       'numMembers', M);



% center of mass projection

% Forwards Kinematics
% length of the members are calculated by using the position of the nodes.
% This is the forward kinematics problem. The solution is found by solving
% the following linear system:
%
%                       L = R * x
%
% where L is the vector of member lengths.

% to find the R matrix, we can first calculate the member lengths in the
% reference configuration.
% L = [norm(trussNodes(1,:) - trussNodes(2,:)); ...
%      norm(trussNodes(2,:) - trussNodes(3,:)); ...
%      norm(trussNodes(3,:) - trussNodes(1,:)); ...

L = zeros(M,1);
for i = 1:M
    L(i) = norm(trussNodes(trussConnectivity(i,1),:) - trussNodes(trussConnectivity(i,2),:));
end


% quick plot
figure(1)
clf
hold on
axis equal
grid on
xlabel('x')
ylabel('y') 
zlabel('z')
view(3)
for i = 1:M
    pause(1)
    plot3(trussNodes(trussConnectivity(i,:),1), ...
          trussNodes(trussConnectivity(i,:),2), ...
          trussNodes(trussConnectivity(i,:),3), ...
          'k-', 'LineWidth', 2)
end

hold off




% Then we can calculate the R matrix by using the following formula:





% Inverse Kinematics











%|─╯└─╯╰┴─╯╰─────────╯╰─╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰────────╯╰──╯┴┴┴┴──╯╰─|











%─────╮╭─────╮╭─────────╮─╮
%─────││─────││─────────││
%─────╯╰─────╯╰─────────╰╯
%───╮╭───╮╭───╮╭───╮╭───╮
%───││───││───││───││───│
%───╰╯───╰╯───╰╯───╰╯───╰
%─────────────────────────
%─╮╭───╮─╮
%││───│─│
%╰╯───╰─╯
%╭─────────╮
%│───╮╭───╮│
%│───││───││
%│───╰╯───││
%╰─────────╯
%─────╮╭─────╮╭─────────╮─╮
%─────││─────││─────────││
%─────╯╰─────╯╰─────────╰╯
%───╮╭───╮╭───╮╭───╮╭───╮
%───││───││───││───││───│
%───╰╯───╰╯───╰╯───╰╯───╰
%─────────────────────────
%─╮╭───╮─╮
%││───│─│
%╰╯───╰─╯
%╭─────────╮
%│───╮╭───╮│
%│───││───││
%│───╰╯───││
%╰─────────╯


%╭─╮╱╱╱╭────╮╱╱╭─╮╱╱╱╱╭───╮
%╰─╯╱╱╱┃╭╮╭╮┃╱╱┃ │╱╱╱╱│   │
%╭─╮╭━━┫╰╯╰╯┣━━╯ │╭━━╯   ╰──╯
%╰─╯┃╭╮┃╭━━╮┃╭━━╯╰━━╮
%╭─╮┃╰╯┃┃   ┃┃╰━━╮╭───╯
%╰─╯╰━━┻╯   ╰┻━━━╯╰───────────────────╮

%┌─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮
%├─╯└─╯├─╯╱╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤└─╯├─╯╱╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤└─╯├─╯╱╱╱╱╱╱╱╱╱╱┃┃╱├┴┐├─╯├─╯├─┤
%┴─╮┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤┌─╮┴─╮╭┬─╮╱╱╱╱╱╱┃┃╱├─┤└─╯├─╮┴─┤
%─╯└─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯
