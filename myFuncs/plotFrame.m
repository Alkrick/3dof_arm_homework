function plotFrame(xyz,R)
%PLOTFRAME Summary of this function goes here
%   Detailed explanation goes here xyz is a column matrix containing x in
%   the first row x values, second row y, third row z. R is a 3x3xn Matrix
%   where n is the rotation matrix of the nth column of xyz

n = size(R,3);
scale = 0.2;
%figure();
plot3(xyz(1,:),xyz(2,:),xyz(3,:),'ko');
hold on
grid on

for i = 1:n
   quiver3(xyz(1,i),xyz(2,i),xyz(3,i),R(1,1,i),R(2,1,i),R(3,1,i),'Color','r','AutoScaleFactor',scale);
   quiver3(xyz(1,i),xyz(2,i),xyz(3,i),R(1,2,i),R(2,2,i),R(3,2,i),'Color','g','AutoScaleFactor',scale);
   quiver3(xyz(1,i),xyz(2,i),xyz(3,i),R(1,3,i),R(2,3,i),R(3,3,i),'Color','b','AutoScaleFactor',scale);
   %axis equal
end
    
end

