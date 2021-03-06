%% Script to check the performance of the trajectory following.
close all
clear
clc

run ControllersDesign.m
run pathGeneration.m
% run ControllerLQR.m
% run robustModel.m
% run windModel.m

warning off

refXdot = 0.4;
t = 200;
step_interval =0.05;
enable_disturbance = 1;
xbDisturbance = 1.5;
ybDisturbance = 1.5;
yawDisturbance = 1.5;
sineAmplitude = 1;
sineFrequency = 5;

Nsim = 100;
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
ybDisturbance_rand = [0; ybDisturbance *(2*rand(Nsim-1,1)-1)];
yawDisturbance_rand = [0; yawDisturbance*(2*rand(Nsim-1,1)-1)];

sineFrequency_rand = [0; sineFrequency + sineFrequency*(2*rand(Nsim-1,1)-1)];

% Inialize vector to contain the data
time_sim = linspace(0,t,t/step_interval+1);
xn_lqr_sim = zeros(Nsim,t/step_interval+1);
yn_lqr_sim = zeros(Nsim,t/step_interval+1);
dist_lqr_sim = zeros(Nsim,t/step_interval+1);

xn_rob_sim = zeros(Nsim,t/step_interval+1);
yn_rob_sim = zeros(Nsim,t/step_interval+1);
dist_rob_sim = zeros(Nsim,t/step_interval+1);

for i=1:1:Nsim
    disp(i)
    if (i==1)
        noise_power_yaw = 0;
        noise_power_yawdot = 0;
        noise_power_xbdot = 0;
    else
        noise_power_yaw = 9.8847e-9;
        noise_power_yawdot = 9.8847e-9;
        noise_power_xbdot = 9.8847e-9;
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
    ybDisturbance = ybDisturbance_rand(i);
    yawDisturbance = yawDisturbance_rand(i);
    sineFrequency = sineFrequency_rand(i);
    
    % Simulate 
    sim PathFollowing.slx
    
    % Get data
    xn_lqr_sim(i,:) = xn_lqr.Data';
    yn_lqr_sim(i,:) = yn_lqr.Data';
    dist_lqr_sim(i,:) = dist_lqr.Data';
    
    xn_rob_sim(i,:) = xn_rob.Data';
    yn_rob_sim(i,:) = yn_rob.Data';
    dist_rob_sim(i,:) = dist_rob.Data';
    
    clear index
    clear index2
end

% save MonteCarloPathNoCorrection Nsim time_sim...
%     m_rand Iz_rand dx_rand dy_rand dyaw_rand l1_rand l2_rand...
%     xbDisturbance_rand yawDisturbance_rand sineFrequency_rand...
%     xn_lqr_sim yn_lqr_sim dist_lqr_sim xn_rob_sim yn_rob_sim...
%     dist_rob_sim

%% Analize results
run ControllersDesign.m
run pathGeneration.m
load MonteCarloPathNoCorrection
%load MonteCarloPath

yn_lqr_sim_min = min(yn_lqr_sim);
yn_lqr_sim_max = max(yn_lqr_sim);
xn_lqr_sim_min = min(xn_lqr_sim);
xn_lqr_sim_max = max(xn_lqr_sim);
dist_lqr_sim_min = min(dist_lqr_sim);
dist_lqr_sim_max = max(dist_lqr_sim);
dist_lqr_sim_sigma = std(dist_lqr_sim);

yn_rob_sim_min = min(yn_rob_sim);
yn_rob_sim_max = max(yn_rob_sim);
xn_rob_sim_min = min(xn_rob_sim);
xn_rob_sim_max = max(xn_rob_sim);
dist_rob_sim_min = min(dist_rob_sim);
dist_rob_sim_max = max(dist_rob_sim);
dist_rob_sim_sigma = std(dist_rob_sim);


yline = [0 2];
xline1 = [49 49];
xline2 = [82 82];
xline3 = [125 125];
xline4 = [156 156];

figure(1)
plot(wps(:,2),wps(:,1),'--xk');
hold on
Y=[xn_lqr_sim_min,fliplr(xn_lqr_sim_max)];              
X=[yn_lqr_sim_max,fliplr(yn_lqr_sim_min)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
h = plot(yn_lqr_sim(1,:),xn_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
X=[yn_lqr_sim_min,fliplr(yn_lqr_sim_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
FigureLatex('Path Following using the Linear Quadratic Regulator','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Deviation Area','Nominal Response'},'NorthEast',[-5 25],[0 30],12,13,0)
uistack(h,'top');

figure(2)
X=[time_sim,fliplr(time_sim)];              
Y=[dist_lqr_sim_max,fliplr(dist_lqr_sim_min)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
hold on
Y=[dist_lqr_sim(1,:) - dist_lqr_sim_sigma,fliplr(dist_lqr_sim(1,:) + dist_lqr_sim_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none'); 
plot(time_sim,dist_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the Linear Quadratic Regulator','Time [s]','Distance [m]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response','Straight Path Limits'},'NorthEast',0,[0 1],12,13,1.1)

yline = [0 2];
xline1 = [51 51];
xline2 = [84 84];
xline3 = [125 125];
xline4 = [158 158];

figure(3)
plot(wps(:,2),wps(:,1),'--xk');
hold on
Y=[xn_rob_sim_min,fliplr(xn_rob_sim_max)];              
X=[yn_rob_sim_max,fliplr(yn_rob_sim_min)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
h = plot(yn_rob_sim(1,:),xn_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
X=[yn_rob_sim_min,fliplr(yn_rob_sim_max)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
FigureLatex('Path Following using the $\mathcal{H}_\infty$ Controller','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Deviation Region','Nominal Response'},'NorthEast',[-5 25],[0 30],12,13,0)
uistack(h,'top');

figure(4)
X=[time_sim,fliplr(time_sim)];              
Y=[dist_rob_sim_max,fliplr(dist_rob_sim_min)];
fill(X,Y,[0.8 0.2 0],'edgecolor','none'); 
hold on
Y=[dist_rob_sim(1,:) - dist_rob_sim_sigma,fliplr(dist_rob_sim(1,:) + dist_rob_sim_sigma)];
fill(X,Y,[0 0.6 0],'edgecolor','none'); 
plot(time_sim,dist_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the $\mathcal{H}_\infty$ Controller','Time [s]','Distance [m]',1,{'Deviation Region','1-$\sigma$ Region','Nominal Response','Straight Path Limits'},'NorthEast',0,[0 1],12,13,1.1)

