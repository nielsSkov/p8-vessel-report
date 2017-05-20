%% Script to check the performance of the trajectory following.
close all
clear
clc
run ControllersDesign.m

warning off

xbdot = 0.5;
t = 40;

sim OuterController.slx

% Analize results
figure(2)
plot(dist.Time,dist.Data)
FigureLatex('Initial Condition Response of the Outer Controller','Time [s]','Distance [m]',0,0,0,0,0,12,13,1.2)
