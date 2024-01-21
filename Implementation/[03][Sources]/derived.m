clc
clear
close

[filename filepath] = uigetfile('sch.png');
[X, map] = imread(filename);
info = imfinfo(filename)
width = getfield(imfinfo(filename),'Width');
height = getfield(imfinfo(filename),'Height');
figure('Name','Display imported image','NumberTitle','off');
image(X);
R = X(:,:,1);
G = X(:,:,2);
B = X(:,:,3);
Rimg = cat(3, R, zeros(size(R)), zeros(size(R)));
Gimg = cat(3, zeros(size(G)), G, zeros(size(G)));
Bimg = cat(3, zeros(size(B)), zeros(size(B)), B);
figure('Name','RED component of the imported image','NumberTitle','off');
image(Rimg);
figure('Name','GREEN component of the imported image','NumberTitle','off');
image(Gimg);
figure('Name','BLUE component of the imported image','NumberTitle','off');
image(Bimg);
G = rgb2gray(X);    
figure('Name','Gray-scale image of the imported image','NumberTitle','off');
imshow(G);