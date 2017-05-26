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
t = 15000;
step_interval =0.05;
enable_disturbance = 1;
xbDisturbance = 1.5;
ybDisturbance = 1.5;
yawDisturbance = 1.5;
sineAmplitude = 1;
sineFrequency = 5;

Nsim = 2;
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

noise_power_yaw = 9.8847e-9;
noise_power_yawdot = 9.8847e-9;
noise_power_xbdot = 9.8847e-9;


%Select the new parameters
m = m_rand(2);
dx = dx_rand(2);
dy = dy_rand(2);
Iz = Iz_rand(2);
dyaw = dyaw_rand(2);
l1 = l1_rand(2);
l2 = l2_rand(2);
xbDisturbance = xbDisturbance_rand(2);
ybDisturbance = ybDisturbance_rand(2);
yawDisturbance = yawDisturbance_rand(2);
sineFrequency = sineFrequency_rand(2);

% Simulate
sim PathFollowingMap.slx

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

%% Analize results
run ControllersDesign.m
run pathGeneration.m

yline = [0 2];
xline1 = [49 49];
xline2 = [82 82];
xline3 = [125 125];
xline4 = [156 156];

figure(1)
plot(wps(:,2),wps(:,1),'--xk');
hold on
plot(yn_lqr_sim(1,:),xn_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
FigureLatex('Path Following using the Linear Quadratic Regulator','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Response'},'NorthEast',0,0,12,13,0)

figure(2)
hold on
plot(time_sim,dist_lqr_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the Linear Quadratic Regulator','Time [s]','Distance [m]',1,{'Response','Straight Path Limits'},'NorthEast',0,[0 1],12,13,1.1)

yline = [0 2];
xline1 = [51 51];
xline2 = [84 84];
xline3 = [125 125];
xline4 = [158 158];

figure(3)
plot(wps(:,2),wps(:,1),'--xk');
hold on
h = plot(yn_rob_sim(1,:),xn_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7]);
FigureLatex('Path Following using the $\mathcal{H}_\infty$ Controller','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Path','Response'},'NorthEast',0,0,12,13,0)
uistack(h,'top');

figure(4)
hold on
plot(time_sim,dist_rob_sim(1,:),'LineWidth',1.5,'Color',[0 0 0.7])
plot(xline1,yline,'--','Color',[0.5 0 0.8])
plot(xline2,yline,'--','Color',[0.5 0 0.8])
plot(xline3,yline,'--','Color',[0.5 0 0.8])
plot(xline4,yline,'--','Color',[0.5 0 0.8])
FigureLatex('Distance to the Path when using the $\mathcal{H}_\infty$ Controller','Time [s]','Distance [m]',1,{'Response','Straight Path Limits'},'NorthEast',0,[0 1],12,13,1.1)


%%
close all
% replace with an image of your choice
img = imread('Captura3.png');
 
% set the range of the axes
% The image will be stretched to this.
min_x = -1300;
max_x = 2300;
min_y = -1100;
max_y = 1100;
% Flip the image upside down before showing it
imagesc([min_x max_x], [min_y max_y], flipud(img));
 
% NOTE: if your image is RGB, you should use flipdim(img, 1) instead of flipud.
 
hold on;
plot(wps(:,2),wps(:,1),'--ok');
hold on
plot(yn_lqr_sim(1,1:end-8000),xn_lqr_sim(1,1:end-8000),'Linewidth',1.2);
 
% set the y-axis back to normal.
set(gca,'ydir','normal');
% xlim([-500 1000])
% ylim([-400 500])