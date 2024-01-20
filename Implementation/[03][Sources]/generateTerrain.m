%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% TERRAIN GENERATION
% From the heightmap, the terrain is generated.

function generateTerrain(image)
global terrain;
% Generate the terrain from the heightmap.
% The terrain is a mesh of triangles.
% The heightmap is a grayscale image.

% Import the image.
image = imread(image);

% Convert the image to grayscale.
image = rgb2gray(image);

imshow(image);

% Get the size of the image.
[height, width] = size(image);

fprintf('Height: %d\n', height);

% Create the terrain.
terrain = zeros(height, width, 3);

% Create the vertices.
for i = 1:height
    for j = 1:width
        % Get the height of the vertex.
        height = image(i, j);
        
        % Create the vertex.
        terrain(i, j, :) = [i, j, height];
    end
end

% Create the faces.
faces = zeros((height - 1) * (width - 1), 3);

% Create the faces.
for i = 1:height - 1
    for j = 1:width - 1
        % Get the indices of the vertices.
        v1 = (i - 1) * width + j;
        v2 = (i - 1) * width + j + 1;
        v3 = i * width + j;
        v4 = i * width + j + 1;
        
        % Create the faces.
        faces((i - 1) * (width - 1) + j, :) = [v1, v2, v3];
        faces((i - 1) * (width - 1) + j + (height - 1) * (width - 1), :) = [v2, v4, v3];
    end
end

% Create the mesh.
terrainMesh = struct('vertices', terrain, 'faces', faces);

% Save the terrain.

fprintf('Saving terrain...\n');

save('terrain.mat', 'terrain');

fprintf('Terrain saved.\n');

% plot the mesh

fprintf('Plotting terrain...\n');

figure;
trisurf(terrainMesh.faces, terrainMesh.vertices(:, 1), terrainMesh.vertices(:, 2), terrainMesh.vertices(:, 3));
xlabel('x');
ylabel('y');
zlabel('z');
title('Terrain');
axis equal;









end



