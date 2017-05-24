%% Script to check the performance of the trajectory following.
close all
clear
clc
run Parameters.m

warning off

xbdot = 0.4;
t = 40;

r = 1;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 1.5;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 2;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 2.5;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 3;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 3.5;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 4;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on

r = 4.5;
R = r;
sim OuterController.slx
figure(1)
plot(dist.Time,dist.Data)
hold on


FigureLatex('Initial Condition Response of the Outer Controller','Time [s]','Distance [m]',1,{'R = 1 m','R = 1.5 m','R = 2 m','R = 2.5 m','R = 3 m','R = 3.5 m','R = 4 m','R = 4.5 m'},'NorthEast',0,[0 1],12,13,1.2)
