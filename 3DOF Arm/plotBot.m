

for i = 1:100 

p = traj(i,:)';
l = [0;1;1];

q = invKine(p,l);

Ax_x = [-2 2];
Ax_y = [-2 2];
Ax_z = [-2 2];

q1 = q(1); q2 = q(2); q3 = q(3);
l1 = l(1); l2 = l(2); l3 = l(3);

Te = double(subs(TMATRIX));
TE = chainMulti(Te,4,1);


P1 = TE(1:3,4,1);
P2 = TE(1:3,4,2);
P3 = TE(1:3,4,3);
P4 = TE(1:3,4,4);

%
plot3(P1(1),P1(2),P1(3),'ko');
plot3(P3(1),P3(2),P3(3),'ko');
plot3(P4(1),P4(2),P4(3),'ko');

plot3([P1(1) P3(1)],[P1(2) P3(2)],[P1(3) P3(3)],'c');
plot3([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)],'m');

plotFrame([P1 P2 P3 P4],TE(1:3,1:3,:));
axis([Ax_x Ax_y Ax_z]);
axis equal;
pause();

end