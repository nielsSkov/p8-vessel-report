%% Verify Turn
close all
clear
clc

run Parameters.m
load dataTurn.mat

t = 8;

sim Model.slx

figure(1)
plot(yn_sim.Data,xn_sim.Data)
hold on
plot(yngps(105:end-100)+493.68,xngps(105:end-100)-64.2,'.-')
grid on
grid minor
axis equal
hold off


%% Verify Step
close all
clear
clc

run Parameters.m
load dataStep.mat

t = 4;
sim Model.slx

figure(1)
plot(yn_sim.Data,xn_sim.Data)
hold on
plot(yngps(115:203)+493.68,xngps(115:203)-64.2,'.-')
grid on
grid minor
axis equal
hold off
