close all
clear
clc

data = csvread('F0003CH1.csv',19,3);
t = data(:,1);
x = data(:,2)*10;

N = 10;
a = 1;
b = 1/N * ones(1,N);

y = filter(b,a,x);

thr=0.1;
med=medfilt2(x,[10,1]); 

figure
hold on
plot(t,y)
grid on 
grid minor
legend('Current drawn by the motor')
%xlim([2.35 5])
%plot(t(x<med-thr),x(x<med-thr))
%plot(t(x>med+thr),x(x>med+thr))
%%
x(x<med-thr)=med(x<med-thr);
x(x>med+thr)=med(x>med+thr);

figure
hold on
plot(t,x)
%plot(t,y)