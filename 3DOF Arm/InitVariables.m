
%% Model Parameters
m1 = 0; m2 = 0.1; m3 = 0.1;  % Masses
l1 = 0; l2 = 1; l3 = 1;  % Link lenths

m = [m1 m2 m3];
l = [l1 l2 l3];

load('TMATRIX.mat');  % T Matrices of all frames (used in ikine)

% Inertia 
I1 = zeros(3);  I2 = 5*eye(3); I3 = 5*eye(3); 
I = cat(3,I1,I2,I3);

g = 9.81;

