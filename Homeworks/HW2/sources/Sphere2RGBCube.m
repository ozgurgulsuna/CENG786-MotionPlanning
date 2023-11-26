function [ RGB ] = Sphere2RGBCube( V )
%Sphere2RGBCube converts the normalized vector V (representing a point on
%the unit spher) into its corresponding RGB cube values. zero vectors are
%outpt as NaN. The accurate version finds the intersection with the RGB
%cube while the "non accurate" version performs a  smoother color
%conversion.
%Author:Itzik Ben Sabat
%Date: 27.1.2016
if size(V,2) > 3
    V = V';
    transposeglag = true;
else 
    transposeglag = false; 
end
RGB = zeros(size(V));
V = V./repmat((sqrt(sum(V.^2,2))),1,3); %make sure V is normalized
%Map from unit cube to RGB Cube 
RGB = 0.5*V+0.5;
RGB(all(isnan(V),2),:)=nan; % zero vectors are mapped to black
if transposeglag
    RGB = RGB';
end
end