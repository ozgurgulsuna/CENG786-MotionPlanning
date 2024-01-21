%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% TERRAIN GENERATION
% From the heightmap, the terrain is generated.
% The terrain is a mesh of triangles.
% The heightmap is a grayscale image.

function generateTerrain(image)
    global terrain;


    % Import the image.
    image = imread(image);

    % convert the image to grayscale
    image = rgb2gray(image);

    % downsampling the image 
    factor = 2;
    image = imresize(image, [size(image,1)/factor size(image,2)/factor]);

    % Get the size of the image.
    [height, width] = size(image);


    % image = rgb2gray(image);
    figure
    imshow(image);


    terrain = zeros(1, 3);

    for x = 1:height
        for y = 1:width
            % Get the height of the point.
            z = image(x, y);
            z = double(z)/20;
            
            % Add the point to the array.
            terrain = [terrain; x y z];
        end
    end

    % convert the uint8 to double
    terrain = double(terrain);

    terrainPt = pointCloud(terrain);

    % % plot the point cloud
    % figure
    % pcshow(terrainPt)

    gridstep = 4;
    ptCloudDownSampled = pcdownsample(terrainPt,"gridAverage",gridstep);

    % figure
    % pcshow(ptCloudDownSampled)

    depth = 4;
    mesh = pc2surfacemesh(ptCloudDownSampled,"ball-pivot");
    figure
    surfaceMeshShow(mesh)

    % save the mesh
    save('terrainMesh.mat', 'mesh');

    % stlwrite('terrain.stl', mesh);


end



