%% IMU test
close all
clear
clc

% Load IMU data
data=csvread('imu_data_060417.csv',11,0);
%load imu_data_06042017

% Get sensor data
xgyro=data(:,3)*pi/180;
xgyro=xgyro(xgyro<10);
xgyro=xgyro(xgyro>-10);
ygyro=-data(:,4)*pi/180;
ygyro=ygyro(ygyro<10);
ygyro=ygyro(ygyro>-20);
zgyro=-data(:,5)*pi/180;
zgyro=zgyro(zgyro<10);
zgyro=zgyro(zgyro>-10);
xacc=data(:,6);
yacc=-data(:,7);
zacc=-data(:,8);
xmag=data(:,9);
ymag=-data(:,10);
zmag=-data(:,11);

% Calculate roll and pitch
roll=-atan2(yacc,sqrt(xacc.^2+zacc.^2));
pitch=-atan2(xacc,sqrt(yacc.^2+zacc.^2));

% Calculate yaw
Mxh=xmag.*cos(pitch)+ymag.*sin(roll).*sin(pitch)+zmag.*cos(roll).*sin(pitch);
Myh=ymag.*cos(roll)+zmag.*sin(roll);
yaw=atan2(Myh,Mxh);

% Calculate variances
sigma2_roll=var(roll)
sigma2_pitch=var(pitch)
sigma2_yaw=var(yaw)

% Calculate variances of the accelerometer
xacc=xacc(xacc<10);
xacc=xacc(xacc>-10);
yacc=yacc(yacc<10);
yacc=yacc(yacc>-10);
zacc=zacc(zacc<10);
zacc=zacc(zacc>-10);
sigma2_xacc=var(xacc)
sigma2_yacc=var(yacc)
sigma2_zacc=var(zacc)

% Calculate variances and biases of the gyroscope
sigma2_xgyro=var(xgyro)
sigma2_ygyro=var(ygyro)
sigma2_zgyro=var(zgyro)

bias_xgyro=mean(xgyro)
bias_ygyro=mean(ygyro)
bias_zgyro=mean(zgyro)

% Plots
FigHandle = figure('Position', [50, 50, 900, 600]);
subplot(3,1,1)
plot(xacc(100:end))
FigureLatex('$\ddot{x}_\mathrm{b}$',0,0,0,0,0,0,0,14,14,1.2)
subplot(3,1,2)
plot(yacc(100:end))
FigureLatex('$\ddot{y}_\mathrm{b}$',0,'Acceleration [m s$^{-2}$]',0,0,0,0,0,14,14,1.2)
subplot(3,1,3)
plot(zacc(100:end))
FigureLatex('$\ddot{z}_\mathrm{b}$','Sample',0,0,0,0,0,0,14,14,1.2)

FigHandle = figure('Position', [50, 50, 900, 600]);
subplot(3,1,1)
plot(xgyro)
FigureLatex('$\dot{\phi}_\mathrm{gyro}$',0,0,0,0,0,0,0,14,14,1.2)
subplot(3,1,2)
plot(ygyro)
FigureLatex('$\dot{\theta}_\mathrm{gyro}$',0,'Angular Velocity [rad/s]',0,0,0,0,0,14,14,1.2)
subplot(3,1,3)
plot(zgyro)
FigureLatex('$\dot{\psi}_\mathrm{gyro}$','Sample',0,0,0,0,0,0,14,14,1.2)

FigHandle = figure('Position', [50, 50, 900, 600]);
subplot(3,1,1)
plot(roll)
FigureLatex('$\phi_\mathrm{acc}$',0,0,0,0,0,0,0,14,14,1.2)
subplot(3,1,2)
plot(pitch)
FigureLatex('$\theta_\mathrm{acc}$',0,'Angular Position [rad]',0,0,0,0,0,14,14,1.2)
subplot(3,1,3)
plot(yaw)
FigureLatex('$\psi_\mathrm{mag}$ ','Sample',0,0,0,0,0,0,14,14,1.2)

