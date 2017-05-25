%% Inner LQR
close all
clear 
clc

% Get data
data_attitude = csvread('2017-05-25_model_test_inner_lqr_att.txt',1,0);
data_input = csvread('2017-05-25_model_test_inner_lqr_input.txt',1,0);
data_position = csvread('2017-05-25_model_test_inner_lqr_pos.txt',1,0);

time_base = data_attitude(:,1)';
n_samples = length(time_base);
Ts = 0.05;
MPOS = 6.6044;
NPOS = 70.0168;
MNEG = 8.5706;
NNEG = 91.9358;

% Extract inputs
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
F1 = fillin(time_base,time_F,F1temp);
F2 = fillin(time_base,time_F,F2temp);
for i = 1:1:length(F1)
    if F1(i) < 0
        F1(i) = (F1(i) - NPOS) / MPOS;
    else
        F1(i) = (F1(i) + NNEG) / MNEG;
    end
    if F2(i) < 0
        F2(i) = (F2(i) - NPOS) / MPOS;
    else
        F2(i) = (F2(i) + NNEG) / MNEG;
    end
end

% Extract velocity and heading
xbdot = fillin(time_base,data_position(:,1),data_position(:,4));
yaw = fillin(time_base,data_attitude(:,1),data_attitude(:,4));
xn = fillin(time_base,data_position(:,1),data_position(:,2));
yn = fillin(time_base,data_position(:,1),data_position(:,3));

t = linspace(0,0.0615*n_samples,n_samples);

figure(1)
plot(t(1:end-1),yaw(1:end-1))
FigureLatex('Response in $\psi$','Time [s]','Angular Position [rad s$^{-1}$]',0,0,0,[0 50],0,12,13,1.2);

figure(2)
plot(t,[0 xbdot(1:end-1)])
FigureLatex('Response in $\dot{x}_\mathrm{b}$','Time [s]','Angular Position [rad s$^{-1}$]',0,0,0,[0 50],0,12,13,1.2);

% Outer LQR
clear 
clc

% Get data
data_attitude = csvread('2017-05-25_model_outer_lqr_att.txt',1,0);
data_input = csvread('2017-05-25_model_outer_lqr_input.txt',1,0);
data_position = csvread('2017-05-25_model_outer_lqr_pos.txt',1,0);
data_wps = csvread('wps.txt');

time_base = data_attitude(:,1)';
n_samples = length(time_base);
Ts = 0.05;
MPOS = 6.6044;
NPOS = 70.0168;
MNEG = 8.5706;
NNEG = 91.9358;

% Extract inputs
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
F1 = fillin(time_base,time_F,F1temp);
F2 = fillin(time_base,time_F,F2temp);
for i = 1:1:length(F1)
    if F1(i) < 0
        F1(i) = (F1(i) - NPOS) / MPOS;
    else
        F1(i) = (F1(i) + NNEG) / MNEG;
    end
    if F2(i) < 0
        F2(i) = (F2(i) - NPOS) / MPOS;
    else
        F2(i) = (F2(i) + NNEG) / MNEG;
    end
end

% Extract velocity and heading
xbdot = fillin(time_base,data_position(:,1),data_position(:,4));
yaw = fillin(time_base,data_attitude(:,1),data_attitude(:,4));
xn = fillin(time_base,data_position(:,1),data_position(:,2));
yn = fillin(time_base,data_position(:,1),data_position(:,3));

t = linspace(0,0.0615*n_samples,n_samples);

figure(3)
plot(data_wps(1:17,2),data_wps(1:17,1),'--xk')
hold on
plot(yn(1:end-500),xn(1:end-500))
FigureLatex(0,'$y_\mathrm{n}$ [m]','$y_\mathrm{n}$ [m]',1,{'Path','Response'},0,[-5 25],0,12,13,1.2);
