%% IMU test
close all
clear
clc

% Load IMU data
load imu_data_06042017

% Get sensor data
xgyro=data(:,3);
xgyro=xgyro(xgyro<10);
xgyro=xgyro(xgyro>-10);
ygyro=data(:,4);
ygyro=ygyro(ygyro<10);
ygyro=ygyro(ygyro>-20);
zgyro=data(:,5);
zgyro=zgyro(zgyro<10);
zgyro=zgyro(zgyro>-10);
xacc=data(:,6);
yacc=data(:,7);
zacc=data(:,8);
xmag=data(:,9);
ymag=data(:,10);
zmag=data(:,11);


bias_xgyro=mean(xgyro);
bias_ygyro=mean(ygyro);
bias_zgyro=mean(zgyro);

% Calculate roll and pitch
roll=atan(yacc./sqrt(xacc.^2+zacc.^2));
pitch=atan(xacc./sqrt(yacc.^2+zacc.^2));

% Calculate yaw
Mxh=xmag.*cos(pitch)+ymag.*sin(roll).*sin(pitch)+zmag.*cos(roll).*sin(pitch);
Myh=ymag.*cos(roll)+zmag.*sin(roll);
yaw=atan(Myh./Mxh);


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

