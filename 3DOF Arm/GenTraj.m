
Ts = resolution;
T = linspace(0,simTime/p_seg,(simTime/p_seg) * fps);
T = T(1:end)';

n = size(T,1);

%% Path Generation


switch pathType
   
    
    case 0
%% Point-2-Point

p_i = [ 1 -1 1]';  %Initial point
p_f = [ 1  1 0]';  %Final point

% Linear interpolation between points 
p_x = linspace(p_i(1),p_f(1),p_seg*n); 
p_y = linspace(p_i(2),p_f(2),p_seg*n);
p_z = linspace(p_i(3),p_f(3),p_seg*n);

% Final Path
p = [p_x;p_y;p_z];

% Compute joint angles for trajectory
q_r=invKine(p,l);

dq_r=zeros(p_seg*n,3);
ddq_r=zeros(p_seg*n,3);

%Visualize  
f1 = figure();
T = tspan;
tf = tspan(end);
q_i = q_r(1,:);
q_f = q_r(end,:);
q_r0=q_r;

subplot(2,2,1)
hold on
grid on
plot([0;0;0],q_i,'bo');
plot([tf;tf;tf],q_f,'ro');
plot(tspan(1:10:end),q_r(1:10:end,:),'ko');
title("Joint Position")
legend('InitPos','FinalPos','ComputedPos')

subplot(2,2,2)
plot(tspan,dq_r);
grid on
title("Joint Velocity")
legend('q1','q2','q3')

subplot(2,2,3)
plot(tspan,ddq_r);
grid on
title("Joint Acceleration")
legend('q1','q2','q3')

if simFlag
subplot(2,2,4)
    %Simulate Trajectory
    kinematicSim;
end

    case 1
   %% Circle

% Circle Parameters
A = 0.3;
b = [0.5 0.5];
fq = 2;
Ph = 0;

% Generate circle points
p_x = A*cos(fq*tspan+Ph)+b(1);
p_y = A*sin(fq*tspan+Ph)+b(2);
p_z = 0.5*ones(size(p_x,2),1)';
p = [p_x;p_y;p_z];

% Compute joint angles
q_r=invKine(p,l);
dq_r=zeros(p_seg*n,3);
ddq_r=zeros(p_seg*n,3);

%Visualize  
f1 = figure();
T = tspan;
tf = tspan(end);
q_i = q_r(1,:);
q_f = q_r(end,:);
q_r0=q_r;

subplot(2,2,1)
hold on
grid on
plot([0;0;0],q_i,'bo');
plot([tf;tf;tf],q_f,'ro');
plot(tspan(1:10:end),q_r(1:10:end,:),'ko');
title("Joint Position")
legend('InitPos','FinalPos','ComputedPos')

subplot(2,2,2)
plot(tspan,dq_r);
grid on
title("Joint Velocity")
legend('q1','q2')

subplot(2,2,3)
plot(tspan,ddq_r);
grid on
title("Joint Acceleration")
legend('q1','q2')

if simFlag
subplot(2,2,4)
    %Simulate Trajectory
    kinematicSim;
end

%% Word Writing
    case 2
% Get Word Trajectory (Currently only supports writing One Word)         
getWord;
n = size(word,2);
tS = size(tspan,2);

q_r=[];
% Linear interpolation between each point in 'Word'
 for i = 1:n-1
    p_i = word(:,i);
    p_f = word(:,i+1);
    
    p_x = linspace(p_i(1),p_f(1),tS/(n-1));
    p_y = linspace(p_i(2),p_f(2),tS/(n-1));
    p_z = linspace(p_i(3),p_f(3),tS/(n-1));
    
    p = [p_x;p_y;p_z];

    q_r=[q_r;invKine(p,l)];
    
 end
dq_r=zeros(tS,3);
ddq_r=zeros(tS,3);
%Visualize  
f1 = figure();
T = tspan;
tf = T(end);
q_i = q_r(1,:);
q_f = q_r(end,:);
q_r0=q_r;

subplot(2,2,1)
hold on
grid on
plot([0;0;0],q_i,'bo');
plot([tf;tf;tf],q_f,'ro');
plot(T,q_r,'ko');
title("Joint Position")
legend('InitPos','FinalPos','ComputedPos')

subplot(2,2,2)
plot(T,dq_r);
grid on
title("Joint Velocity")
legend('q1','q2','q3')

subplot(2,2,3)
plot(T,ddq_r);
grid on
title("Joint Acceleration")
legend('q1','q2','q3')

if simFlag
subplot(2,2,4)
    %Simulate Trajectory
    kinematicSim;
end
        
end

disp("Finished Trajectory Generation.")
pause();