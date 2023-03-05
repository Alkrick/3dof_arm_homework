clear;
close all;

addpath ../myFuncs

%% Init Variables
pathType=1; % Try 0,1,2 

InitVariables;
simParams;

%% Generate Trajectory
GenTraj;

close all;

f1 = figure('Name','Angle');
grid on 
hold on

plot(tspan,q_r(:,1)','r');
plot(tspan,q_r(:,2)','b');
plot(tspan,q_r(:,3)','g');
legend q_r1 q_r2 q_r3

%% Control Loop

q = q_r(1,:);
dq = [0 0 0];
Q = 0;

fExt = [0 0 0 0 0 0]';

z0 = [q dq];
zf=z0;

kp = 3500;
kd = 259.8; 
ki = -25;    

options=odeset('abstol',2.5e-14,'reltol',2.5e-14);
f4=figure('Name','Bot');
pause();
  
line=[];

 
for i = 1:size(tspan,2)-1
    %% PD Controller
    qerr = q_r(i,:) - q;
    %qerr = qref - q;
    
    uerr = dq_r(i,:) - dq;
    %uerr = uref - dq;
    
    Qerr = 0 - Q;

    tau = kp * qerr + kd * uerr + ki * Qerr;
                 
    [~, z] = ode113('NewtonRHS3',[tspan(i) tspan(i+1)],z0,options, ...
                              m, I, l, g,tau,fExt);

    %Set state for next iteration
    q = [z(end,1) z(end,2) z(end,3)];  
    dq = [z(end,4) z(end,5) z(end,6)];
    Q = Q + qerr;
    
    %For Visualization
    z0 = [z(end,1) z(end,2) z(end,3) z(end,4) z(end,5) z(end,6)];
    zf = [zf; z0];

    figure(f4);
    grid on
    Ax_x = [-2 2];
    Ax_y = [-2 2];
    Ax_z = [-2 2];

    q1 = q(1); q2 = q(2); q3 = q(3);

    Te = double(subs(TMATRIX));
    TE = chainMulti(Te,4,1);

    P1 = TE(1:3,4,1);
    P2 = TE(1:3,4,2);
    P3 = TE(1:3,4,3);
    P4 = TE(1:3,4,4);
    
    line = [line; P4'];

    %
    plot3(P1(1),P1(2),P1(3),'ko');
    hold on
    grid on
    plot3(P3(1),P3(2),P3(3),'ko');
    plot3(P4(1),P4(2),P4(3),'ko');
    

    plot3([P1(1) P3(1)],[P1(2) P3(2)],[P1(3) P3(3)],'c','LineWidth',1.5);
    plot3([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)],'m','LineWidth',1.5);
    plot3(line(:,1),line(:,2),(line(:,3)),'Color','#0072BD','LineStyle','-');
    xlabel(['time = ',num2str(tspan(i))]);
    
    %plotFrame([P1 P2 P3 P4],TE(1:3,1:3,:));
    axis([Ax_x Ax_y Ax_z]);
    axis square;
    hold off
    
end

%% Visualize Control
figure(f1)
grid on 
hold on
plot(tspan,zf(:,1)','r--');
plot(tspan,zf(:,2)','b--');
plot(tspan,zf(:,3)','g--');
legend q_r_1 q_r_2 q_r_3 q_1 q_2 q_3;


