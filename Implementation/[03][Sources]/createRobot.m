%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% CREATE ROBOT
% The robot created can have different number of nodes and members. Selected topology is octahedron.
% Initial length of the members is 1 meters, it is presumed that the average lenght will stay the same.

function createRobot()

    global robotTopology;

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
                        ]  ;



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
                    ] ;

    robotTopology = struct('nodes', [trussNodes], ...
                        'connectivity', [trussConnectivity], ...
                        'members', [trussMembers] );




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
