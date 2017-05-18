
close all
clear
clc

run ControllersDesign.m

warning off

refYaw = 1;
refXdot = 1;
t = 50;
step_interval =0.05;
time_step = 10;
enable_disturbance = 1;
xbDisturbance = 1.5;
yawDisturbance = 1.5;
sineAmplitude = 0.5;
sineFrequency = 5;

Nsim = 1;
% Create random vectors for the parameters and the disturbances
percentage = 0.2;
m_rand = [m; m + (percentage*m*(2*rand(Nsim-1,1)-1))];
Iz_rand = [Iz; Iz + (percentage*Iz*(2*rand(Nsim-1,1)-1))];
Iz_rand(Iz_rand<0) = Iz;

dx_rand = [dx; dx + (percentage*dx*(2*rand(Nsim-1,1)-1))];
dx_rand(dx_rand<0) = dx;
dy_rand = [dy; dy + (percentage*dy*(2*rand(Nsim-1,1)-1))];
dy_rand(dy_rand<0) = dy;
dyaw_rand = [dyaw; dyaw + (percentage*dyaw*(2*rand(Nsim-1,1)-1))];
dyaw_rand(dyaw_rand<0) = dyaw;

l1_rand = [l1; l1 + (percentage*l1*(2*rand(Nsim-1,1)-1))];
l2_rand = [l2; l2 + (percentage*l2*(2*rand(Nsim-1,1)-1))];

xbDisturbance_rand = [0; xbDisturbance *(2*rand(Nsim-1,1)-1)];
yawDisturbance_rand = [0; yawDisturbance*(2*rand(Nsim-1,1)-1)];

sineFrequency_rand = [0; sineFrequency + sineFrequency*(2*rand(Nsim-1,1)-1)];

% Inialize vector to contain the data
time_sim = linspace(0,t,t/step_interval+1);
xbdot_lqr_sim = zeros(Nsim,t/step_interval+1);
yaw_lqr_sim = zeros(Nsim,t/step_interval+1);
yawdot_lqr_sim = zeros(Nsim,t/step_interval+1);
xbdot_int_lqr_sim = zeros(Nsim,t/step_interval+1);
yaw_int_lqr_sim = zeros(Nsim,t/step_interval+1);
F1_lqr_sim = zeros(Nsim,t/step_interval+1);
F2_lqr_sim = zeros(Nsim,t/step_interval+1);

xbdot_rob_sim = zeros(Nsim,t/step_interval+1);
yaw_rob_sim = zeros(Nsim,t/step_interval+1);
yawdot_rob_sim = zeros(Nsim,t/step_interval+1);
xbdot_int_rob_sim = zeros(Nsim,t/step_interval+1);
yaw_int_rob_sim = zeros(Nsim,t/step_interval+1);
F1_rob_sim = zeros(Nsim,t/step_interval+1);
F2_rob_sim = zeros(Nsim,t/step_interval+1);

for i=1:1:Nsim
    disp(i)
    if (i==1)
        noise_power_yaw = 0;
        noise_power_yawdot = 0;
        noise_power_xbdot = 0;
    else
        noise_power_yaw = 0.0000001;
        noise_power_yawdot = 0.000001;
        noise_power_xbdot = 0.000001;
    end
    %Select the new parameters
    m = m_rand(i);
    dx = dx_rand(i);
    dy = dy_rand(i);
    Iz = Iz_rand(i);
    dyaw = dyaw_rand(i);
    l1 = l1_rand(i);
    l2 = l2_rand(i);
    xbDisturbance = xbDisturbance_rand(i);
    yawDisturbance = yawDisturbance_rand(i);
    sineFrequency = sineFrequency_rand(i);
    
    % Simulate controllers
    sim('InnerController.slx')
    
    % Store the results
    xbdot_lqr_sim(i,:) = xbdot_lqr.Data';
    yaw_lqr_sim(i,:) = yaw_lqr.Data';
    yawdot_lqr_sim(i,:) = yawdot_lqr.Data';
    xbdot_int_lqr_sim(i,:) = xbdot_int_lqr.Data';
    yaw_int_lqr_sim(i,:) = yaw_int_lqr.Data';
    F1_lqr_sim(i,:) = F1_lqr.Data';
    F2_lqr_sim(i,:) = F2_lqr.Data';
    
    xbdot_rob_sim(i,:) = xbdot_rob.Data';
    yaw_rob_sim(i,:) = yaw_rob.Data';
    yawdot_rob_sim(i,:) = yawdot_rob.Data';
    xbdot_int_rob_sim(i,:) = xbdot_int_rob.Data';
    yaw_int_rob_sim(i,:) = yaw_int_rob.Data';
    F1_rob_sim(i,:) = F1_rob.Data';
    F2_rob_sim(i,:) = F2_rob.Data';
    
end

% save MonteCarloControllers Nsim time_sim xbdot_lqr_sim yaw_lqr_sim...
%     F1_lqr_sim F2_lqr_sim xbdot_rob_sim yaw_rob_sim F1_rob_sim F2_rob_sim...
%     mx_rand Iz_rand dx_rand dy_rand dyaw_rand l1_rand l2_rand...
%     xbDisturbance_rand yawDisturbance_rand sineFrequency_rand...
%     yawdot_lqr_sim yawdot_rob_sim yaw_int_lqr_sim xbdot_int_lqr_sim...
%     yaw_int_rob_sim xbdot_int_rob_sim

%% Analize resultsrun ControllersDesign.m
run ControllersDesign.m
load MonteCarloControllers

cost_lqr = zeros(Nsim,1);
cost_rob = zeros(Nsim,1);

for i=1:1:Nsim
    % Calculate cost for each simulation
    for j = 1:1:size(xbdot_lqr_sim,2)
        cost_lqr(i) = cost_lqr(i)...
            + [yaw_lqr_sim(i,j) yawdot_lqr_sim(i,j) xbdot_lqr_sim(i,j) yaw_int_lqr_sim(i,j) xbdot_int_lqr_sim(i,j)]...
            * Q * [yaw_lqr_sim(i,j); yawdot_lqr_sim(i,j); xbdot_lqr_sim(i,j); yaw_int_lqr_sim(i,j); xbdot_int_lqr_sim(i,j)]...
            + [F1_lqr_sim(i,j) F2_lqr_sim(i,j)] * R * [F1_lqr_sim(i,j); F2_lqr_sim(i,j)];
        
        cost_rob(i) = cost_rob(i)...
            + [yaw_rob_sim(i,j) yawdot_rob_sim(i,j) xbdot_rob_sim(i,j) yaw_int_rob_sim(i,j) xbdot_int_rob_sim(i,j)]...
            * Q * [yaw_rob_sim(i,j); yawdot_rob_sim(i,j); xbdot_rob_sim(i,j); yaw_int_rob_sim(i,j); xbdot_int_rob_sim(i,j)]...
            + [F1_rob_sim(i,j) F2_rob_sim(i,j)] * R * [F1_rob_sim(i,j); F2_rob_sim(i,j)];
    end
end

xbdot_lqr_max = max(xbdot_lqr_sim);
xbdot_lqr_min = min(xbdot_lqr_sim);
xbdot_rob_max = max(xbdot_rob_sim);
xbdot_rob_min = min(xbdot_rob_sim);
yaw_lqr_max = max(yaw_lqr_sim);
yaw_lqr_min = min(yaw_lqr_sim);
yaw_rob_max = max(yaw_rob_sim);
yaw_rob_min = min(yaw_rob_sim);

xbdot_lqr_sigma = std(xbdot_lqr_sim);
xbdot_rob_sigma = std(xbdot_rob_sim);
yaw_lqr_sigma = std(yaw_lqr_sim);
yaw_rob_sigma = std(yaw_rob_sim);

% Plot the results
figure(1)
X=[time_sim,fliplr(time_sim)];
Y=[xbdot_lqr_min,fliplr(xbdot_lqr_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none');
hold on
X=[time_sim,fliplr(time_sim)];
Y=[xbdot_lqr_sim(1,:) - xbdot_lqr_sigma,fliplr(xbdot_lqr_sim(1,:) + xbdot_lqr_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none');
plot(time_sim,xbdot_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])

figure(2)
Y=[xbdot_rob_min,fliplr(xbdot_rob_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none');
hold on
Y=[xbdot_rob_sim(1,:) - xbdot_rob_sigma,fliplr(xbdot_rob_sim(1,:) + xbdot_rob_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none');
plot(time_sim,xbdot_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])

figure(3)
Y=[yaw_lqr_min,fliplr(yaw_lqr_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none');
hold on
Y=[yaw_lqr_sim(1,:) - yaw_lqr_sigma,fliplr(yaw_lqr_sim(1,:) + yaw_lqr_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none');
plot(time_sim,yaw_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])

figure(4)
Y=[yaw_rob_min,fliplr(yaw_rob_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none');
hold on
Y=[yaw_rob_sim(1,:) - yaw_rob_sigma,fliplr(yaw_rob_sim(1,:) + yaw_rob_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none');
plot(time_sim,yaw_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])

figure(1)
FigureLatex('Step Response in $\dot{x}_\mathrm{b}$ of the Linear Quadratic Regulator','Time [s]','Translational Velocity [m s$^-1$]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response'},'SouthEast',[0 30],[-0.1 1.5],12,13,0)
figure(2)
FigureLatex('Step Response in $\dot{x}_\mathrm{b}$ of the $\mathcal{H}_\infty$ Controller','Time [s]','Translational Velocity [m s$^-1$]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response'},'SouthEast',[0 30],[-0.1 1.5],12,13,1.1)
figure(3)
FigureLatex('Step Response in $\psi$ of the Linear Quadratic Regulator','Time [s]','Angular Position [rad]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response'},'SouthEast',[0 30],[-0.1 1.5],12,13,1.1)
figure(4)
FigureLatex('Step Response in $\psi$ of the $\mathcal{H}_\infty$ Controller','Time [s]','Angular Position [rad]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response'},'SouthEast',[0 30],[-0.1 1.5],12,13,1.1)

% Calculate mean cost between the simulations
mean_cost_lqr = mean(cost_lqr)
mean_cost_rob = mean(cost_rob)

% Calculate mean input at each moment
mean_input_lqr = mean(mean(abs(F1_lqr_sim)+abs(F2_lqr_sim),1))
mean_input_rob = mean(mean(abs(F1_rob_sim)+abs(F2_rob_sim),1))