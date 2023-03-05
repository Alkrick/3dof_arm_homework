function [f,n] = inwardIter(R,P,Pc,F,N,fExt)
%INWARDITER Summary of this function goes here
%   Detailed explanation goes here


l=size(R,3)-1;

f(:,l)= fExt(1:3)+F(:,l);
n(:,l)= fExt(4:6)+...
        N(:,l)+...
        cross( Pc(:,:,l), f(:,l) )+...
        cross( P(:,:,l+1)-Pc(:,:,l) , R(:,:,l+1)*fExt(1:3) );

for i = fliplr(1:l-1)
    f(:,i)= R(:,:,i+1)*f(:,i+1)+F(:,i);
    
    n(:,i)= N(:,i)+...
            R(:,:,i+1)*n(:,i+1)+...
            cross( Pc(:,:,i), f(:,i) )+...
            cross( P(:,:,i+1)-Pc(:,:,i) , R(:,:,i+1)*f(:,i+1) );

end

