close all
clear
clc

data = csvread('PRINT_04.csv',2,0);
t = data(:,1);
x = data(:,2);

N = 10;
a = 1;
b = 1/N * ones(1,N);

y = filter(b,a,x);

thr=0.1;
med=medfilt2(x,[10,1]); 

figure
hold on
plot(t,x)
plot(t(x<med-thr),x(x<med-thr))
plot(t(x>med+thr),x(x>med+thr))
%%
x(x<med-thr)=med(x<med-thr);
x(x>med+thr)=med(x>med+thr);

figure
hold on
plot(t,x)
%plot(t,y)