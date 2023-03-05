clear;
addpath ../myFuncs

%% Define Variables

syms q1 q2 q3 u1 u2 u3 ud1 ud2 ud3 real
syms l1 l2 l3 m1 m2 m3 T1 T2 T3 g real
syms fx fy fz nx ny nz real
syms I1 I1_1 I1_2 I1_3 I1_12 I1_13 I1_23 real
syms I2 I2_1 I2_2 I2_3 I2_12 I2_13 I2_23 real
syms I3 I3_1 I3_2 I3_3 I3_12 I3_13 I3_23 real
syms x y z real

I1=[I1_1  I1_12 I1_13;
    I1_12 I1_2  I1_23;
    I1_13 I1_23 I1_3];
I2=[I2_1  I2_12 I2_13;
    I2_12 I2_2  I2_23;
    I2_13 I2_23 I2_3];
I3=[I3_1  I3_12 I3_13;
    I3_12 I3_2  I3_23;
    I3_13 I3_23 I3_3];

% I1 = zeros(3); 
% I2 = zeros(3);  

I = cat(3,I1,I2,I3);
m = [m1 m2 m3]';

q = [q1; q2; q3];       %Joint Rotation
u = [u1; u2; u3; 0];    %Joint Velocity
ud = [ud1; ud2; ud3; 0]; %Joint Acceleration
      
Torque = [T1 T2 T3]';

goal = [x; y; z];  %Goal point in 3D space

%% Define Model
%    theta   d   a   alpha 
DH = [q1     0   0   0;
      q2     0   0  pi/2;
      q3     0   l2  0;
      0      0   l3  0];
  
%% Kinematics 

% Function that returns T matrices of each frame as 4x4xn matrix 
% (where n is numer of frames)
T = DH2T(DH);

P = T(1:3,4,:);   % Origins of each frame
R = T(1:3,1:3,:); % Rotations between frames

% All T and R matrices expressed in World frame
Tw = simplify(chainMulti(T,4,1)); 
Rw = Tw(1:3,1:3,:);

R01 = Rw(:,:,1);
R02 = Rw(:,:,2);
R03 = Rw(:,:,3);

T01 = Tw(:,:,1);
T02 = Tw(:,:,2);
T03 = Tw(:,:,3);

% Origin in world and CoM of each frame  
[Pw, Pc] = getWorldP(R,P);

% CoM of each frame expressed in world frame
P1g = R01*Pc(:,:,1);
P2g = R02*Pc(:,:,2)+P1g;
P3g = R03*Pc(:,:,3)+P2g;
Pcg = cat(3,P1g,P1g,P2g,P3g);


%% Jacobians

% Function that calculates geometric Velocity Jacobian of each frame
[Jv1, Jw1]=myJacobian(Pw,Rw,q,2);
[Jv2, Jw2]=myJacobian(Pw,Rw,q,3);
[Jv3, Jw3]=myJacobian(Pw,Rw,q,4);

% Jacobian of CoM
[Jv_c1, Jw_c1]=myJacobian(Pcg,Rw,q,2);
[Jv_c2, Jw_c2]=myJacobian(Pcg,Rw,q,3);
[Jv_c3, Jw_c3]=myJacobian(Pcg,Rw,q,4);

%J = [Jv ; Jw];

%% Dynamics

% Outward Iterations to calculate velocities and accelerations
a0 = [0 0 g]';
[v,vc,w,vd,wd,ac,F,N]=outwardIter(R,P,Pc,u,ud,a0,m,I);  

fExt = [fx fy fz 0 0 0]'; % External force on end affector
fExt0 = [0 0 0 0 0 0]';

% Inward Iteration to calculate Forces and Torques
[f,n] = inwardIter(R,P,Pc,F,N,fExt);
[f0,n0] = inwardIter(R,P,Pc,F,N,fExt0);

tau = simplify(n'*[0 0 1]');
tau0 = simplify(n0'*[0 0 1]');


% Velocities and accelerations of all links expressed in their respective
% frames
V = [v;  w];
A = [vd; wd];
Ac = simplify(ac); 

% Velocities and accelerations of all links expressed in World frame
v_w = simplify([R01*v(:,1) R02*v(:,2) R03*v(:,3) R03*v(:,4)]);
w_w = simplify([R01*w(:,1) R02*w(:,2) R03*w(:,3) R03*w(:,4)]);

% Velocities of CoM 
Vg1 = v(:,1) + cross( w(:,1), P1g );
Vg2 = v(:,2) + cross( w(:,2), P2g );
Vg3 = v(:,3) + cross( w(:,3), P3g );


%% Newton-Euler

eqn = collect(simplify(Torque-tau),[ud1 ud2 ud3]);

eqn1 = eqn(1);
eqn2 = eqn(2);
eqn3 = eqn(3);


%% Lagrange 

u  = u(1:3);
ud = ud(1:3);

%
Ke1 = 0.5*m1* u'*(Jv_c1'*Jv_c1)*u + 0.5*u'*Jw_c1'*R01*I1*R01'*Jw_c1*u;
Ke2 = 0.5*m2* u'*(Jv_c2'*Jv_c2)*u + 0.5*u'*Jw_c2'*R02*I2*R02'*Jw_c2*u;
Ke3 = 0.5*m3* u'*(Jv_c3'*Jv_c3)*u + 0.5*u'*Jw_c3'*R03*I3*R03'*Jw_c3*u;

Pe1 = 0.5*m1*dot(a0,P1g);
Pe2 = 0.5*m2*dot(a0,P2g);
Pe3 = 0.5*m3*dot(a0,P3g);

KE = simplify(Ke1+Ke2+Ke3);
PE = simplify(Pe1+Pe2+Pe3);

L = KE - PE;
Ld_qd = jacobian(L,u);
Ldd_qd_dt = simplify(jacobian(Ld_qd,q)*u+jacobian(Ld_qd,u)*ud);

Ld_q = jacobian(L,q);

L1 = T1-Ldd_qd_dt(1)+Ld_q(1);
L2 = T2-Ldd_qd_dt(2)+Ld_q(2);
L3 = T3-Ldd_qd_dt(3)+Ld_q(3);

% eqn1 = collect(simplify(L1),[ud1 ud2 ud3]);
% eqn2 = collect(simplify(L2),[ud1 ud2 ud3]);
% eqn3 = collect(simplify(L3),[ud1 ud2 ud3]);


%% Output 

RHS1 = -subs(eqn1,[ud1 ud2 ud3],[0 0 0]); %negative sign because we want to take RHS to the right eventually
M11  =  subs(eqn1,[ud1 ud2 ud3],[1 0 0]) + RHS1;
M12  =  subs(eqn1,[ud1 ud2 ud3],[0 1 0]) + RHS1;
M13  =  subs(eqn1,[ud1 ud2 ud3],[0 0 1]) + RHS1;

RHS2 = -subs(eqn2,[ud1 ud2 ud3],[0 0 0]);
M21  =  subs(eqn2,[ud1 ud2 ud3],[1 0 0]) + RHS2;
M22  =  subs(eqn2,[ud1 ud2 ud3],[0 1 0]) + RHS2;
M23  =  subs(eqn2,[ud1 ud2 ud3],[0 0 1]) + RHS2;

RHS3 = -subs(eqn3,[ud1 ud2 ud3],[0 0 0]);
M31  =  subs(eqn3,[ud1 ud2 ud3],[1 0 0]) + RHS3;
M32  =  subs(eqn3,[ud1 ud2 ud3],[0 1 0]) + RHS3;
M33  =  subs(eqn3,[ud1 ud2 ud3],[0 0 1]) + RHS3;%


%%%%%%% Final system [M][alpha] = [RHS] %%%%%%%%%%%

M   = [M11 M12 M13; M21 M22 M23; M31 M32 M33];
RHS = [RHS1; RHS2; RHS3];

%% Inverse Dynamics

B1 = m1*(Jv_c1'*Jv_c1)+Jw_c1'*R01*I1*R01'*Jw_c1;
B2 = m2*(Jv_c2'*Jv_c2)+Jw_c2'*R02*I2*R02'*Jw_c2;
B3 = m3*(Jv_c3'*Jv_c3)+Jw_c3'*R03*I3*R03'*Jw_c3;

B = simplify(B1+B2+B3);

G = subs(tau,[ud1 ud2 ud3 u1 u2 u3 fx fy fz nz],[0 0 0 0 0 0 0 0 0 0]);

T_ext = simplify(tau-tau0);

c111 = 1 / 2 * diff(B(1, 1), q1);
c112 = 1 / 2 * diff(B(1, 1), q2);
c113 = 1 / 2 * diff(B(1, 1), q3);
c121 = c112;
c122 = diff(B(1, 2), q2) - 1 / 2 * diff(B(2, 2), q1);
c123 = 1 / 2 * ( diff(B(1,2),q3) + diff(B(1,3),q2) - diff(B(2,3),q1));
c131 = c113;
c132 = c123;
c133 = diff(B(1, 3), q3) - 1 / 2 * diff(B(3, 3), q1);

c211 = diff(B(2, 1), q1) - 1 / 2 * diff(B(1, 1), q2);
c212 = 1 / 2 * diff(B(2, 2), q1);
c213 = 1/2 * ( diff(B(2,1),q3)+ diff(B(2,3),q1)- diff(B(1,3),q2));
c221 = c212;
c222 = 1 / 2 * diff(B(2, 2), q2);
c223 = 1/2*(diff(B(2,2),q3));
c231 = 213;
c232 = 223;
c233 = diff(B(2, 3), q3) - 1 / 2 * diff(B(3, 3), q2);

c311 = diff(B(3, 1), q1) - 1 / 2 * diff(B(1, 1), q3);
c312 = 1/2 * ( diff(B(3,1),q2)+ diff(B(3,2),q1)- diff(B(1,2),q3));
c313 = 1 / 2 * diff(B(3, 3), q1);
c321 = c312;
c322 = diff(B(3, 2), q2) - 1 / 2 * diff(B(2, 2), q3);
c323 = 1 / 2 * diff(B(3, 3), q2);
c331 = c313;
c332 = c323;
c333 = 1 / 2 * diff(B(3, 3), q3);

c11 = c111*u1 + c112*u2 + c113*u3; 
c12 = c121*u1 + c122*u2 + c123*u3; 
c13 = c131*u1 + c132*u2 + c133*u3; 

c21 = c211*u1 + c212*u2 + c213*u3;
c22 = c221*u1 + c222*u2 + c223*u3;
c23 = c231*u1 + c232*u2 + c233*u3;

c31 = c311*u1 + c312*u2 + c313*u3;
c32 = c321*u1 + c322*u2 + c323*u3;
c33 = c331*u1 + c332*u2 + c333*u3;

C = [c11 c12 c13;
     c21 c22 c23;
     c31 c32 c33];
  
%%% Derivation complete %%%
 
fid=fopen(   'NewtonRHS3.m','w');
fprintf(fid, 'function zdot=NewtonRHS3(t,z,flag,m,I, l, g, tau, fExt)   \n\n');
 
fprintf(fid, 'q1 = z(1);   u1 = z(4);                         \n');
fprintf(fid, 'q2 = z(2);   u2 = z(5);                         \n');
fprintf(fid, 'q3 = z(3);   u3 = z(6);                         \n\n');

fprintf(fid, 'm1 = m(1);   m2 = m(2);  m3 = m(3);             \n');
fprintf(fid, 'l1 = l(1);   l2 = l(2);  l3 = l(3);             \n');
fprintf(fid, 'I1 = I(:,:,1);   I2 = I(:,:,2);  I3 = I(:,:,3); \n\n');

fprintf(fid, 'fx = fExt(1); fy = fExt(2); fz = fExt(3);        \n');
fprintf(fid, 'nx = fExt(4); ny = fExt(5); nz = fExt(6);        \n\n');

fprintf(fid, 'I1_1 = I1(1,1);I1_2 = I1(2,2);I1_3 = I1(3,3);    \n');
fprintf(fid, 'I1_12 = I1(1,2);I1_13 = I1(1,3);I1_23 = I1(2,3); \n\n');

fprintf(fid, 'I2_1 = I2(1,1);I2_2 = I2(2,2);I2_3 = I2(3,3);    \n');
fprintf(fid, 'I2_12 = I2(1,2);I2_13 = I2(1,3);I2_23 = I2(2,3); \n\n');

fprintf(fid, 'I3_1 = I3(1,1);I3_2 = I3(2,2);I3_3 = I3(3,3);    \n');
fprintf(fid, 'I3_12 = I3(1,2);I3_13 = I3(1,3);I3_23 = I3(2,3); \n\n');

fprintf(fid, 'T1 = tau(1); T2 = tau(2); T3 = tau(3);\n');

fprintf(fid,'M11 = %s; \n', char((M(1,1))) );
fprintf(fid,'M12 = %s; \n', char((M(1,2))) );
fprintf(fid,'M13 = %s; \n', char((M(1,3))) );
fprintf(fid,'\n');
 
fprintf(fid,'M21 = %s; \n', char((M(2,1))) );
fprintf(fid,'M22 = %s; \n', char((M(2,2))) );
fprintf(fid,'M23 = %s; \n', char((M(2,3))) );
fprintf(fid,'\n');

fprintf(fid,'M31 = %s; \n', char((M(3,1))) );
fprintf(fid,'M32 = %s; \n', char((M(3,2))) );
fprintf(fid,'M33 = %s; \n', char((M(3,3))) );
fprintf(fid,'\n');

fprintf(fid,'RHS1 = %s; \n', char((RHS(1))) );
fprintf(fid,'RHS2 = %s; \n', char((RHS(2))) );
fprintf(fid,'RHS3 = %s; \n', char((RHS(3))) );
fprintf(fid,'\n');
 

fprintf(fid,'MM  = [M11 M12 M13;                               \n');
fprintf(fid,'       M21 M22 M23;                               \n');
fprintf(fid,'       M31 M32 M33];                              \n\n');
 
fprintf(fid,'RHS = [RHS1; RHS2; RHS3];                      \n\n');
 
fprintf(fid,'X = MM \\ RHS;                                    \n\n');
 
fprintf(fid,'ud1 = X(1);                                       \n');
fprintf(fid,'ud2 = X(2);                                     \n\n');
fprintf(fid,'ud3 = X(3);                                     \n\n');
 
fprintf(fid,'zdot = [u1 u2 u3 ud1 ud2 ud3]'';\n'); 
fclose(fid);

