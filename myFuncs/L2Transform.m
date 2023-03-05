function [T] = L2Transform(L)
%TRANSFORMF Summary of this function goes here
%   Detailed explanation goes here
% L = [th d alpha a]
th = L(1);
d = L(2);
a = L(3);
alph = L(4);

T = [    cos(th)          -sin(th)            0             a;
     sin(th)*cos(alph) cos(th)*cos(alph) -sin(alph) -sin(alph)*d;
     sin(th)*sin(alph) cos(th)*sin(alph)  cos(alph)  cos(alph)*d;
            0                 0               0             1];
end

