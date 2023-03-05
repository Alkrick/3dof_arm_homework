function [Rn] = chainMulti(R,n,flag)
%POSITIONVECS Summary of this function goes here
%   Detailed explanation goes here
if flag
    Rn = [];
    for i = 1:n
        Rn = cat(3,Rn,chainMulti(R,i,0));
    end    
else
    Rn = eye(size(R,1));
    for i = 1:n
        Rn=Rn*R(:,:,i);
    end
end

end

