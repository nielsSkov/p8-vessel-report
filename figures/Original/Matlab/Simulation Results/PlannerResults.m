%% Script to check the performance of the trajectory following.
close all
clear
clc

run ControllersDesign.m
run pathGeneration.m

refXdot = 0.5;
t = 4000;
step_interval =0.05;
enable_disturbance = 1;
xbDisturbance = 1.5;
ybDisturbance = 1.5;
yawDisturbance = 1.5;
sineAmplitude = 1;
sineFrequency = 10;

% Inialize vector to contain the time
time_sim = linspace(0,t,t/step_interval+1);

noise_power_yaw = 0;%9.8847e-9;
noise_power_yawdot = 0;%9.8847e-9;
noise_power_xbdot = 0;%9.8847e-9;

% Simulate
sim SimulationResults.slx

% Get data
xn_lqr_sim = xn_lqr.Data';
yn_lqr_sim = yn_lqr.Data';
dist_lqr_sim = dist_lqr.Data';

xn_rob_sim = xn_rob.Data';
yn_rob_sim = yn_rob.Data';
dist_rob_sim = dist_rob.Data';


% save MonteCarloPathNoCorrection Nsim time_sim...
%     m_rand Iz_rand dx_rand dy_rand dyaw_rand l1_rand l2_rand...
%     xbDisturbance_rand yawDisturbance_rand sineFrequency_rand...
%     xn_lqr_sim yn_lqr_sim dist_lqr_sim xn_rob_sim yn_rob_sim...
%     dist_rob_sim

% Analize results
run ControllersDesign.m
run pathGeneration.m
%load MonteCarloPathNoCorrection
%load MonteCarloPath

yline = [0 2];
xline1 = [40 40];
xline2 = [65 65];
xline3 = [99 99];
xline4 = [125 125];

figure(1)
plot(wps(:,2),wps(:,1),'--xk');
hold on
plot(yn_lqr_sim(1,:),xn_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
FigureLatex('Path Following using the Linear Quadratic Regulator','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Response'},'NorthEast',0,0,12,13,0)


figure(2)
plot(time_sim,dist_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
hold on
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the Linear Quadratic Regulator','Time [s]','Distance [m]',1,{'Response','Straight Path Limits'},'NorthEast',0,0,12,13,1.1)

yline = [0 2];
xline1 = [42 42];
xline2 = [68 68];
xline3 = [102 102];
xline4 = [128 128];

figure(3)
plot(wps(:,2),wps(:,1),'--xk');
hold on
plot(yn_rob_sim(1,:),xn_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
FigureLatex('Path Following using the $\mathcal{H}_\infty$ Controller','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Response'},'NorthEast',0,0,12,13,0)


figure(4)
hold on
plot(time_sim,dist_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the $\mathcal{H}_\infty$ Controller','Time [s]','Distance [m]',1,{'Response','Straight Path Limits'},'NorthEast',0,0,12,13,0)

