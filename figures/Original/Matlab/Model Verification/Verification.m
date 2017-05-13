%% Verify Turn
close all
clear
clc

run Parameters.m
load dataTurn.mat

yn = yngps(105:end-100);
xn = xngps(105:end-100);
F1 = F1(105:end-100);
F2 = F2(105:end-100);

samples = length(yn);

t = 8;
Ts = t /samples;
t = t + 8.2 + 1; 
yaw_init = pi;
sim Model.slx

%%
xn_model = xn_sim.Data(333:end-39);
yn_model = yn_sim.Data(333:end-39);
time_model = yn_sim.Time(333:end-39)-yn_sim.Time(333);


figure(1)
subplot(2,1,1)
plot(time_model*1.5,xn_model,'Color',[0 0 0.6])
hold on
plot(time_model,xn-64.2,'Color',[0.6 0 0])
FigureLatex(0,0,'$x_\mathrm{n} [m]$',1,{'Model','Real Data'},0,[0 4],0,12,13,1.2)
subplot(2,1,2)
plot(time_model*1.5,yn_model,'Color',[0 0 0.6])
hold on
plot(time_model,yn+493.68,'Color',[0.6 0 0])
FigureLatex(0,'Time [s]','$y_\mathrm{n}$ [m]',1,{'Model','Real Data'},'NorthEast',[0 4],0,12,13,1.2)

xn_model = xn_model(1:120);
yn_model = yn_model(1:120);

figure(2)
plot(yn_model-0.1,xn_model-0.1,'Color',[0 0 0.6])
hold on
plot(yn(1:200)+493.68,xn(1:200)-64.2,'.-','Color',[0.6 0 0])
axis equal
FigureLatex(0,'$y_\mathrm{n} [m]$','$x_\mathrm{n} [m]$',1,{'Model','Real Data'},0,0,0,12,13,1.2)


figure(3)
subplot(2,1,1)
plot(time_model,xn-64.2,'Color',[0.6 0 0])
FigureLatex(0,0,'$x_\mathrm{n} [m]$',0,0,0,[0 4],0,12,13,1.2)
subplot(2,1,2)
plot(time_model,yn+493.68,'Color',[0.6 0 0])
FigureLatex(0,'Time [s]','$y_\mathrm{n}$ [m]',0,0,0,[0 4],0,12,13,1.2)

xn_model = xn_model(1:120);
yn_model = yn_model(1:120);

figure(4)
plot(yn(1:200)+493.68,xn(1:200)-64.2,'.-','Color',[0.6 0 0])
axis equal
FigureLatex(0,'$y_\mathrm{n} [m]$','$x_\mathrm{n} [m]$',0,0,0,0,0,12,13,1.2)

figure(5)
plot(time_model,F1,'Color',[0 0 0.6])
hold on
plot(time_model,F2,'Color',[0.6 0 0])
FigureLatex(0,'Time [s]','Force [N]',1,{'F1','F2'},'SouthWest',[0 4],0,12,13,1.2)