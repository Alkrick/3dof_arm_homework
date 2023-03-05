function [Jv, Jw] = myJacobian(Pw,Rw,q,n)
%MYJACOBIAN Summary of this function goes here
%   Detailed explanation goes here
Jv = jacobian(Pw(:,:,n),q);
m = size(Pw,3)-1;
Jw = q(1)*zeros([3 (m-1)]);

for i=1:m
    if n-i>0
        Jw(:,i)=Rw(:,3,i);
    else
       Jw(:,i)=zeros([3 1]);
    end
end

end

