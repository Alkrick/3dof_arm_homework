
t_interval = 0.2; % seconds
p_interval = 11; %
p_seg = p_interval-1;


simTime = 5;
simFlag = 0;  %Set to 1 to see a kinematic simulation

fps = 68;
resolution = 1/fps; %
tspan = linspace(0,simTime,simTime*fps);