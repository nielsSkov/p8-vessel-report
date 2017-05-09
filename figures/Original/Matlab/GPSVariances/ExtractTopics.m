close all
clear 
clc

% Get data
data_gps = csvread('2017-05-04_still_test_gps.csv',1,0);

time_base = data_gps(:,1)';
n_samples = length(time_base);

% Get the GPS data
xngps = fillin(time_base,data_gps(:,1),data_gps(:,3));
yngps = fillin(time_base,data_gps(:,1),data_gps(:,4));
xngps = xngps(1500:end);
yngps = yngps(1500:end);

% Plots
FigHandle = figure('Position', [50, 50, 900, 600]);
subplot(2,1,1)
plot(xngps(1:end-1))
FigureLatex('$x_\mathrm{n}$',0,0,0,0,0,0,[55 65],14,14,1.2)
subplot(2,1,2)
plot(yngps(1:end-1))
FigureLatex('$y_\mathrm{n}$','Sample',0,0,0,0,0,[-60 -50],14,14,1.2)
