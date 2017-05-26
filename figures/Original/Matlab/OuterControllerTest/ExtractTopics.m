close all
clear 
clc

% Get data
data_attitude = csvread('good_controller_katt.txt',1,0);    % kf_attitude topic
data_position = csvread('good_controller_kfs.txt',1,0);     % KF_satesnew topic    
data_input = csvread('good_controller_lli.txt',1,0);        % lli_input topic
data_ref = csvread('xbdotgood_ref.txt',1,0);                % control_reference topic

M = 6.6044;
N = 70.0168;
M_ = 8.5706;
N_ = 91.9368;

time_base = data_attitude(:,1)';
n_samples = length(time_base);
G = 9.81;

% Extract info for each motor
for i=1:1:size(data_input,1)
    if (data_input(i,3)==5)
        if (i==1 || i==2)
            F1temp = data_input(i,4);
            time_F = data_input(i,1);
        else
            F1temp = [F1temp data_input(i,4)];
            time_F = [time_F data_input(i,1)];
        end
    else
        if (i==1 || i==2)
            F2temp = data_input(i,4);
        else
            F2temp = [F2temp data_input(i,4)];
        end
    end
end

% Get the input vector as long as the time_base
F1 = fillin(time_base,time_F,F1temp);
F2 = fillin(time_base,time_F,F2temp);

for i=1:1:n_samples
    if (F1(i)<0)
        F1(i)= (F1(i) + N_) / M_;
    else
        F1(i) = (F1(i) - N) / M;
    end
    if (F2(i)<0)
        F2(i)= (F2(i) + N_) / M_;
    else
        F2(i) = (F2(i) - N) / M;
    end
end

% Get yaw and speed
yaw = fillin(time_base,data_attitude(:,1),data_attitude(:,4));
xbd = fillin(time_base,data_position(:,1),data_position(:,10));

% Get reference
yaw_ref = fillin(time_base,data_ref(:,1),data_ref(:,3));
xbdot_ref = fillin(time_base,data_ref(:,1),data_ref(:,2));
time_base = (time_base(:)-time_base(1))*1e-9;

% Some plots
figure
plot(time_base(1:end-5),yaw(1:end-5))
hold on
yaw_filtered(1) = yaw(1);
for i = 2:1:n_samples
    yaw_filtered(i) = yaw(i) * 0.05 + (1-0.05) * yaw_filtered(i-1);
end
plot(time_base(1:end-5),yaw_filtered(1:end-5))
plot(time_base(1:end-5),yaw_ref(1:end-5))
FigureLatex('$\psi$ Response','Time [s]','Angular Position [rad]',0,0,0,[0 20],0,12,13,1.2)

figure
xbd_filtered(1) = xbd(1);
for i = 2:1:n_samples
    xbd_filtered(i) = xbd(i) * 0.01 + (1-0.01) * xbd_filtered(i-1);
end
plot(time_base(1:end-5),xbd(1:end-5))
hold on
plot(time_base(1:end-5),xbd_filtered(1:end-5))
plot(time_base(1:end-5),xbdot_ref(1:end-5))
FigureLatex('$\dot{x}_\mathrm{b}$ Response','Time [s]','Translational Velocity [m s$^{-1}$]',0,0,0,[0 20],0,12,13,1.2)
hold on