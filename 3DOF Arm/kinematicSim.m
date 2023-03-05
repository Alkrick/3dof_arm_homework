
Ax_x = [-1 1];
Ax_y = [-1 1];
Ax_z = [-1 1];

for t = 1:size(T,2)
    % Current Instant Joint Values
    
    % Draw
    if mod(t,50)==0
        
        q1 = q_r0(t,1);
        q2 = q_r0(t,2);
        q3 = q_r0(t,3);
        
        load('TMATRIX.mat');     

        Te = double(subs(TMATRIX));
        TE = chainMulti(Te,4,1);
        
        figure(f1);
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
        pause(0.1)
    end
    
end