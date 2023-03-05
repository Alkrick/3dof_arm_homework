function [Pw,Pc] = getWorldP(R,P)
%GETWORLDP Summary of this function goes here
%   Detailed explanation goes here

Pw = P(:,:,1);
Pc = [];

for i = 2:size(R,3)
    Pw = cat(3, Pw, simplify( chainMulti(R,i-1,0)*P(:,:,i) )+Pw(:,:,i-1) );         
    Pc = cat(3, Pc,  0.5*P(:,:,i) );
end

end

