%╭─╮╭─╮╭─╮╱╱╱╱╱╱╱╱╱╭╮╱╭┬─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╭─╮┬─╮┌─╱╱╱╱╱╱╱╱╱┌─╮╭─╮╱╱╱╱╭─╮╭─╮╭─╮┬─╮┌─╮╭─╮╱╱╱╱╱╱╱╱╱╭┬─╮┬─╮
%  └─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─╯─╯┴─╯╰┴─╯╰──╯╰─╯─╯──────────╯┴─╯╰┴─╯╰─╯─╯┴─╯╰┴─╯╰─────────╯╱╰─╯┴──╯╰─|
% TERRAIN GENERATION
% From the heightmap, the terrain is generated. The terrain is a mesh of triangles. The heightmap is a gray
% scale image. The image size is 200x150

function generateTerrain(image)
    global terrain;
    global mesh;


    % Import the image.
    image = imread(image);

    % convert the image to grayscale
    image = rgb2gray(image);

    % downsampling the image 
    factor = 1;
    image = imresize(image, [size(image,1)/factor size(image,2)/factor]);

    % Get the size of the image.
    [height, width] = size(image);


    % image = rgb2gray(image);
    figure
    imshow(image);


    terrain = [];

    for x = 1:height
        for y = 1:width
            % Get the height of the point.
            z = image(x, y);
            z = double(z)/100;
            
            % Add the point to the array.
            terrain = [terrain; x/2 y/2 z];
        end
    end

    % convert the uint8 to double
    terrain = double(terrain);

    terrainPt = pointCloud(terrain);

    % % plot the point cloud
    % figure
    % pcshow(terrainPt)

    gridstep = 2;
    ptCloudDownSampled = pcdownsample(terrainPt,"gridAverage",gridstep);

    % figure
    % pcshow(ptCloudDownSampled)

    depth = 4;
    mesh = pc2surfacemesh(ptCloudDownSampled,"ball-pivot");
    figure
    surfaceMeshShow(mesh)
    hold on
    pcshow(ptCloudDownSampled)

    % save the mesh
    save('terrainMesh.mat', 'mesh');

    % stlwrite('terrain.stl', mesh);


end



