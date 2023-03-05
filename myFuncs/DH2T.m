function [T] = DH2T(DH)
%DH2T Summary of this function goes here
%   Detailed explanation goes here
T = L2Transform(DH(1,:));
for i = 1:size(DH,1)-1
    T = cat(3,T,L2Transform(DH(i+1,:)));
end

end

