function [v, vc, w,vd, wd, ac,F,N] = outwardIter(R,P, Pc,dth, ddth, a0,m,I)
%VELOCITY Returns the velocities of each joint with respect to its own
%frame and the velocities of the tool frame with respect to base frame
%   Detailed explanation goes here


% base frame's linear & angular velocity
w(:,1) = [0, 0, dth(1)].';
v(:,1) = dth(1)*[0, 0, 0].';
vc(:,1) = dth(1)*[0, 0, 0].';

% base frame's linear & angular acceleration
wd(:,1) = [0 0 ddth(1)]';
vd(:,1) = R(:,:,1)'*a0 + [0 0 0]';


for i = 2:size(R,3)
    % frame i+1's angular velocity
    w(:,i) = R(:,:,i).'*w(:,i-1)+dth(i)*[0,0,1].';   
    
    % frame i+1's linear velocity
    v(:,i) = R(:,:,i).'*(v(:,i-1)+cross(w(:,i-1),P(:,:,i)));
    
    % frame i+1's linear velocity
    vc(:,i) = R(:,:,i).'*(v(:,i-1)+cross(w(:,i-1),Pc(:,:,i-1)));
    
    % frame i+1's angular acceleration
    wd(:,i) = R(:,:,i).'*wd(:,i-1) + ...
        cross( R(:,:,i).'*w(:,i-1) , dth(i)*[0;0;1] ) + ddth(i)*[0;0;1];
    
    % frame i+1's linear acceleration
    vd(:,i) = R(:,:,i).'*( cross( wd(:,i-1) , P(:,:,i) ) +...
        cross( w(:,i-1) , cross( w(:,i-1) , P(:,:,i) ) ) + vd(:,i-1) ) ;
    
    % link CoM acceleration
    ac(:,i-1)= cross( wd(:,i-1), Pc(:,:,i-1) ) + ...
        cross( w(:,i-1) , cross( w(:,i-1) , Pc(:,:,i-1) ) ) + vd(:,i-1);
    
    % Force and Torque of each link
    F(:,i-1) = m(i-1)*ac(:,i-1);
    N(:,i-1) = I(:,:,i-1)*wd(:,i-1)+cross(w(:,i-1),I(:,:,i-1)*w(:,i-1));
    

end
w=simplify(expand(w));
v=simplify(expand(v));
vc=simplify(expand(vc));
wd=simplify(expand(wd));
vd=simplify(expand(vd));
ac=simplify(expand(ac));
F=simplify(expand(F));
N=simplify(expand(N));

end

