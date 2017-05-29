%% Attitude Kalman filter
close all
clear
clc

run Parameters.m
roll_init=0;
pitch_init=0;
yaw_init=0;
t=60;
sim('KalmanModelSim2.slx')

% Position model
a1=1;
a2=0;
a3=0;
a4=1;

A_pos=[1 0 Ts*a1 Ts*a2 0 0;
     0 1 Ts*a3 Ts*a4 0 0;
    0 0 1 0 Ts 0;
    0 0 0 1 0 Ts;
    0 0 -dx/mx 0 -dx/mx*Ts 0;
    0 0 0 -dy/my 0 -dy/my*Ts];

B_pos=[0 0;
    0 0;
    0 0;
    0 0;
    1/mx 1/mx;
    0 0];

C_pos=[1 0 0 0 0 0;
    0 1 0 0 0 0;
    0 0 0 0 1 0;
    0 0 0 0 0 1];

% State variances
sigma2_xn=0.001;
sigma2_yn=0.001;
sigma2_xndot=0.00001;
sigma2_yndot=0.00001;
sigma2_xnddot=0.00001;
sigma2_ynddot=0.00001;

% Sensor variance
sigma2_gps_xn=0.00853392;
sigma2_gps_yn=0.00662973;
sigma2_acc_xbddot=0.00050346;
sigma2_acc_ybddot=0.00057036;

% Covariances matrices
Q=diag([sigma2_xn, sigma2_yn, sigma2_xndot,sigma2_yndot, sigma2_xnddot,...
    sigma2_ynddot]);
R=diag([sigma2_gps_xn, sigma2_gps_yn, sigma2_acc_xbddot, sigma2_acc_ybddot]);

% Data
x_data=x.Data';
xbdot_data=xbdot.Data';
xbddot_data=xbddot.Data';
att_data=att.Data';
roll=att_data(1,:);
pitch=att_data(2,:);
yaw=att_data(3,:);

n_samples=size(x_data,2);
n=size(A_pos,1);
nu=size(B_pos,2);
nz=size(C_pos,1);

% Add noise to measurements
x_noise=x_data+diag(sqrt([sigma2_xn,sigma2_yn]))*randn(2,n_samples);
xddot_noise=xbddot_data+diag(sqrt([sigma2_acc_xbddot,sigma2_acc_ybddot]))*randn(2,n_samples);

meas=[x_noise;
    xddot_noise];
states=[x_data;
    xbdot_data;
    xbddot_data];

u=u.Data';

% Initialization
x00=zeros(n,1);
u00=[0;0];
P00=eye(n);
x_pos=zeros(n,n_samples);

% First Prediction
x_pos(:,1)=A_pos*x00+B_pos*u00;%+sqrt(Q)*randn(n,1);
P=A_pos*P00*A_pos'+Q;

% Loop
for k=2:1:n_samples-1
    % Approximate A_kal using the previous estimation of the angles
    a1=cos(pitch(k-1))*cos(yaw(k-1));
    a2=sin(roll(k-1))*sin(pitch(k-1))*cos(yaw(k-1))-cos(roll(k-1))*sin(yaw(k-1));
    a3=cos(pitch(k-1))*sin(yaw(k-1));
    a4=sin(roll(k-1))*sin(pitch(k-1))*sin(yaw(k-1))+cos(roll(k-1))*cos(yaw(k-1));
    
    A_pos=[1 0 Ts*a1 Ts*a2 0 0;
        0 1 Ts*a3 Ts*a4 0 0;
        0 0 1 0 Ts 0;
        0 0 0 1 0 Ts;
        0 0 -dx/mx 0 -dx/mx*Ts 0;
        0 0 0 -dy/my 0 -dy/my*Ts];
    
    % Update
    K=P*C_pos'/(C_pos*P*C_pos'+R);
    x_pos(:,k)=x_pos(:,k)+K*(meas(:,k)-C_pos*x_pos(:,k));
    P=(eye(n)-K*C_pos)*P;
    
    % Prediction
    x_pos(:,k+1)=A_pos*x_pos(:,k)+B_pos*u(:,k);%+sqrt(Q)*randn(n,1);
    P=A_pos*P*A_pos'+Q;
end

% Plots
% xn
figure
hold on
plot(x.Time,meas(1,:),'Color',[0 0.7 0])
plot(x.Time,x.Data(:,1),'Color',[0 0 0.7])
plot(x.Time,x_pos(1,:),'Color',[0.7 0 0])
FigureLatex('$x_\mathrm{n}$','Time [s]','Translational Position [m]',1,{'Measurement', 'Estimation', 'Real'},0,[0 20],0,12,14,1.2)

% xb velocity
figure
hold on
plot(x.Time,x_pos(3,:),'Color',[0.7 0 0])
plot(x.Time,xbdot.Data(:,1),'Color',[0 0.7 0])
FigureLatex('$\dot{x}_\mathrm{b}$','Time [s]','Translational Velocity [m s$^{-1}$]',1,{'Estimation', 'Real'},0,[0 40],0,12,14,1.2)

% xb acceleration
figure
hold on
plot(x.Time,meas(3,:),'Color',[0 0.7 0])
plot(x.Time,x_pos(5,:),'Color',[0.7 0 0])
plot(x.Time,xbddot.Data(:,1),'Color',[0 0 0.7])
FigureLatex('$\ddot{x}_\mathrm{b}$','Time [s]','Translational Acceleration [m s$^{-2}$]',1,{'Measurement','Estimation', 'Real'},0,[0 40],0,12,14,1.2)

% xn-yn
figure
hold on
plot(meas(2,1:end/3),meas(1,1:end/3),'Color',[0 0.7 0])
plot(x.Data(1:end/3,2),x.Data(1:end/3,1),'Color',[0 0 0.7])
plot(x_pos(2,1:end/3),x_pos(1,1:end/3),'Color',[0.7 0 0])
FigureLatex('$x_\mathrm{n}$-$y_\mathrm{n}$','$y_\mathrm{n}$ [m]','$x_\mathrm{n}$ [m]',1,{'Measurement', 'Estimation', 'Real'},'NorthEast',0,0,12,14,1.2)
