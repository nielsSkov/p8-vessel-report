%% Attitude Kalman filter
close all
clear
clc

run Parameters.m
roll_init=0;
pitch_init=0;
yaw_init=0;
t=60;
sim('KalmanModelSim.slx')

% Attitude model
A_att=[1 0 0 Ts 0 0 0 0 0;
    0 1 0 0 Ts 0 0 0 0;
    0 0 1 0 0 Ts 0 0 0;
    0 0 0 1 0 0 Ts 0 0;
    0 0 0 0 1 0 0 Ts 0;
    0 0 0 0 0 1 0 0 Ts;
    0 0 0 -droll/Ix 0 0 -Ts*droll/Ix 0 0;
    0 0 0 0 -dpitch/Iy 0 0 -Ts*dpitch/Iy 0;
    0 0 0 0 0 -dyaw/Iz 0 0 -Ts*dyaw/Iz];

B_att=[0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    l1/Iz -l2/Iz];
C_att=[eye(6) zeros(6,3)];

% State variances
sigma2_roll=Ts^2;
sigma2_pitch=Ts^2;
sigma2_yaw=Ts^2;
sigma2_rolldot=0.01*Ts;
sigma2_pitchdot=0.01*Ts;
sigma2_yawdot=0.01*Ts;
sigma2_rollddot=0.001;
sigma2_pitchddot=0.001;
sigma2_yawddot=0.001;

% Sensor variance
sigma2_acc_roll=0.01033366;
sigma2_acc_pitch=3.98535412;
sigma2_mag_yaw=0.03143813;
sigma2_gyro_roll=0.00001165;
sigma2_gyro_pitch=0.00002264;
sigma2_gyro_yaw=0.00779067;
bias_gyro_roll=-0.03753197;
bias_gyro_pitch=-4.49556388;
bias_gyro_yaw=0.16183879;

% Covariances matrices
Q=diag([sigma2_roll, sigma2_pitch, sigma2_yaw, sigma2_rolldot, sigma2_pitchdot,...
    sigma2_yawdot, sigma2_rollddot, sigma2_pitchddot, sigma2_yawddot]);
R=diag([sigma2_acc_roll, sigma2_acc_pitch, sigma2_mag_yaw, sigma2_gyro_roll,...
    sigma2_gyro_pitch, sigma2_gyro_yaw]);

% Data
att_data=att.Data';
attdot_data=attdot.Data';
attddot_data=attddot.Data';
roll=att_data(1,:);
pitch=att_data(2,:);
yaw=att_data(3,:);

n_samples=size(att_data,2);
n=size(A_att,1);
nu=size(B_att,2);
nz=size(C_att,1);

% Add noise to measurements
att_noise=att_data+diag(sqrt([sigma2_acc_roll,sigma2_acc_pitch,sigma2_mag_yaw]))*...
    randn(3,n_samples);
attdot_noise=attdot_data+diag(sqrt([sigma2_gyro_roll,sigma2_gyro_pitch,...
    sigma2_gyro_yaw]))*randn(3,n_samples);

meas=[att_noise;
    attdot_noise];
states=[att_data;
    attdot_data;
    attddot_data];

u=u.Data';

% Initialization
x00=zeros(n,1);
u00=[0;0];
P00=eye(n);
x_att=zeros(n,n_samples);

% First Prediction
x_att(:,1)=A_att*x00+B_att*u00;%+sqrt(Q)*randn(n,1);
P=A_att*P00*A_att'+Q;

% Loop
for k=1:1:n_samples-1
    % Update
    K=P*C_att'/(C_att*P*C_att'+R);
    x_att(:,k)=x_att(:,k)+K*(meas(:,k)-C_att*x_att(:,k));
    P=(eye(n)-K*C_att)*P;
      
    % Prediction
    x_att(:,k+1)=A_att*x_att(:,k)+B_att*u(:,k);%+sqrt(Q)*randn(n,1);
    P=A_att*P*A_att'+Q;
end

% Plots
% Yaw
figure
hold on
plot(att.Time,meas(3,:),'Color',[0 0 0.6])
plot(att.Time,x_att(3,:),'Color',[0.7 0 0])
plot(att.Time,att.Data(:,3),'Color',[0 0.7 0])
FigureLatex('$\psi$','Time [s]','Angular Position [rad]',1,{'Measurement', 'Estimation', 'Real'},0,0,0,12,14,1.2)

% Yaw velocity
figure
hold on
plot(att.Time,meas(6,:),'Color',[0 0 0.6])
plot(att.Time,x_att(6,:),'Color',[0.7 0 0])
plot(att.Time,attdot.Data(:,3),'Color',[0 0.7 0])
FigureLatex('$\dot{\psi}$','Time [s]','Angular Velocity [rad $^{-1}$]',1,{'Measurement', 'Estimation', 'Real'},0,0,0,12,14,1.2)

% Yaw acceleration
figure
hold on
plot(att.Time,x_att(9,:),'Color',[0.7 0 0])
plot(att.Time,attddot.Data(:,3),'Color',[0 0.7 0])
FigureLatex('$\ddot{\psi}$','Time [s]','Angular Acceleration [rad s$^{-2}$]',1,{'Estimation', 'Real'},0,0,0,12,14,1.2)


%% Test with real IMU data%% IMU test
close all
clear
clc

run Parameters.m
% data=csvread('imu_data_060417.csv',1,0);
load imu_data_06042017

% Attitude model
A_att=[1 0 0 Ts 0 0 0 0 0;
    0 1 0 0 Ts 0 0 0 0;
    0 0 1 0 0 Ts 0 0 0;
    0 0 0 1 0 0 Ts 0 0;
    0 0 0 0 1 0 0 Ts 0;
    0 0 0 0 0 1 0 0 Ts;
    0 0 0 -droll/Ix 0 0 -Ts*droll/Ix 0 0;
    0 0 0 0 -dpitch/Iy 0 0 -Ts*dpitch/Iy 0;
    0 0 0 0 0 -dyaw/Iz 0 0 -Ts*dyaw/Iz];

B_att=[0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    0 0;
    l1/Iz -l2/Iz];
C_att=[eye(6) zeros(6,3)];

% State variances
sigma2_roll=Ts^2;
sigma2_pitch=Ts^2;
sigma2_yaw=Ts^2;
sigma2_rolldot=0.01*Ts;
sigma2_pitchdot=0.01*Ts;
sigma2_yawdot=0.01*Ts;
sigma2_rollddot=0.001;
sigma2_pitchddot=0.001;
sigma2_yawddot=0.001;

% Sensor variance
sigma2_acc_roll=0.01033366;
sigma2_acc_pitch=3.98535412;
sigma2_mag_yaw=0.01;%0.03143813;
sigma2_gyro_roll=0.00001165;
sigma2_gyro_pitch=0.00002264;
sigma2_gyro_yaw=0.00779067;
bias_gyro_roll=-0.03753197;
bias_gyro_pitch=-4.49556388;
bias_gyro_yaw=0.16183879;

% Covariances matrices
Q=diag([sigma2_roll, sigma2_pitch, sigma2_yaw, sigma2_rolldot, sigma2_pitchdot,...
    sigma2_yawdot, sigma2_rollddot, sigma2_pitchddot, sigma2_yawddot]);
R=diag([sigma2_acc_roll, sigma2_acc_pitch, sigma2_mag_yaw, sigma2_gyro_roll,...
    sigma2_gyro_pitch, sigma2_gyro_yaw]);

% Get sensor data
xgyro=data(4000:7999,3)'-bias_gyro_roll;
ygyro=data(4000:7999,4)'-bias_gyro_pitch;
zgyro=data(4000:7999,5)'-bias_gyro_yaw;
xacc=data(4000:7999,6)';
yacc=data(4000:7999,7)';
zacc=data(4000:7999,8)';
xmag=data(4000:7999,9)';
ymag=data(4000:7999,10)';
zmag=data(4000:7999,11)';

% Calculate roll and pitch
roll=atan(yacc./sqrt(xacc.^2+zacc.^2));
pitch=atan(xacc./sqrt(yacc.^2+zacc.^2));

% Calculate yaw
Mxh=xmag.*cos(pitch)+ymag.*sin(roll).*sin(pitch)+zmag.*cos(roll).*sin(pitch);
Myh=ymag.*cos(roll)+zmag.*sin(roll);
yaw=atan(Myh./Mxh);

n_samples=size(xgyro,2);
n=size(A_att,1);
nu=size(B_att,2);
nz=size(C_att,1);

meas=[roll;
    pitch;
    yaw;
    xgyro;
    ygyro;
    zgyro];

% Initialization
x00=zeros(n,1);
u00=[0;0];
P00=eye(n);
x_att=zeros(n,n_samples);

% First Prediction
x_att(:,1)=A_att*x00+B_att*u00;%+sqrt(Q)*randn(n,1);
P=A_att*P00*A_att'+Q;

% Loop
for k=1:1:n_samples-1
    % Update
    K=P*C_att'/(C_att*P*C_att'+R);
    x_att(:,k)=x_att(:,k)+K*(meas(:,k)-C_att*x_att(:,k));
    P=(eye(n)-K*C_att)*P;
      
    % Prediction
    x_att(:,k+1)=A_att*x_att(:,k)+B_att*u00;%+sqrt(Q)*randn(n,1);
    P=A_att*P*A_att'+Q;
end

% Plots
time_samples=linspace(0,Ts*n_samples,n_samples);

% Yaw
figure
hold on
plot(time_samples,meas(3,:),'Color',[0 0 0.6])
plot(time_samples,x_att(3,:),'Color',[0.7 0 0])
FigureLatex('$\psi$','Time [s]','Angular Position [rad]',1,{'Measurement', 'Estimation'},0,0,0,12,14,1.2)

% Yaw velocity
figure
hold on
plot(time_samples,meas(6,:),'Color',[0 0 0.6])
plot(time_samples,x_att(6,:),'Color',[0.7 0 0])
FigureLatex('$\dot{\psi}$','Time [s]','Angular Velocity [rad $^{-1}$]',1,{'Measurement', 'Estimation'},0,0,0,12,14,1.2)

% Yaw acceleration
figure
hold on
plot(time_samples,x_att(9,:),'Color',[0.7 0 0])
FigureLatex('$\ddot{\psi}$','Time [s]','Angular Acceleration [rad s$^{-2}$]',1,{'Estimation'},0,0,0,12,14,1.2)
