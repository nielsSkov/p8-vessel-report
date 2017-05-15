close all
clear
clc

% Test data
F_ = [-1150 -1120 -1010 -910 -900 -810 -810 -750 -690 -660 -600 -570 -500 -520 -500];
F_ = F_ * 0.00980665 / 2;
PWM_ = [-140 -138 -134 -132 -130 -128 -126 -124 -122 -120 -118 -116 -114 -112 -110];
F = [700 760 800 850 900 950 1100 1100];
F = F * 0.00980665 / 2;
PWM = [92 94 96 98 100 102 104 106];

% Linear approximation
x_ = linspace(-7,-1,100);
y_ = x_ * 4.2853 * 2 - 91.936;

x = linspace(2, 7,100);
y = x * 3.3022 * 2 + 70.017;

figure(1)
plot(x_,y_)
hold on
scatter(F_,PWM_,'*')
FigureLatex(0,'Force [N]','PWM',1,{'Linear Approximation','Samples'},'SouthEast',[-8 0],0,12,13,1.2)

figure(2)
plot(x,y)
hold on
scatter(F,PWM,'*')
FigureLatex(0,'Force [N]','PWM',1,{'Linear Approximation','Samples'},'SouthEast',[0 8],0,12,13,1.2)