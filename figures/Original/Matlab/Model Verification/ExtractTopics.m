close all
clear 
clc

% Get data
data_imu = csvread('20170511_test4imu.txt',1,0);
data_gps = csvread('20170511_test4gps.txt',1,0);
data_attitude = csvread('20170511_test4kfatt.txt',1,0);
data_position = csvread('20170511_test4kfpos.txt',1,0);
data_input = csvread('20170511_test4input.txt',1,0);
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

% Get the imu data as long as the time_base
xgyro = fillin(time_base,data_imu(:,1),data_imu(:,3));
ygyro = -fillin(time_base,data_imu(:,1),data_imu(:,4));
zgyro = -fillin(time_base,data_imu(:,1),data_imu(:,5));
xacc = fillin(time_base,data_imu(:,1),data_imu(:,6));
yacc = -fillin(time_base,data_imu(:,1),data_imu(:,7));
zacc = -fillin(time_base,data_imu(:,1),data_imu(:,8));
xmag = fillin(time_base,data_imu(:,1),data_imu(:,9));
ymag = fillin(time_base,data_imu(:,1),data_imu(:,10));
zmag = fillin(time_base,data_imu(:,1),data_imu(:,11));

% Get attitude raw calculations
rollacc=-atan2(yacc,sqrt(xacc.^2+zacc.^2));
pitchacc=-atan2(xacc,sqrt(yacc.^2+zacc.^2));

% Calculate yaw
Mxh=xmag.*cos(pitchacc)+ymag.*sin(rollacc).*sin(pitchacc)+zmag.*cos(rollacc).*sin(pitchacc);
Myh=ymag.*cos(rollacc)+zmag.*sin(rollacc);
yawmag=atan2(Myh,Mxh);

% yawmag(1)=atan2(Myh(1),Mxh(1));
% for i=2:1:n_samples
%     yawmag(i) = atan2(Myh(i),Mxh(i));
%     if (((yawmag(i-1)-yawmag(i)) < - pi) || ((yawmag(i-1)-yawmag(i)) > pi))
%         if (yawmag(i) < 0)
%             yawmag(i) = yawmag(i) + 2 * pi;
%         else
%             yawmag(i) = yawmag(i) - 2 * pi;
%         end
%     end
% end

% Correct 
xacc = -(xacc - (G * -sin(pitchacc)));
yacc = -(yacc - (G * sin(rollacc) .* cos(pitchacc)));
zacc = -(zacc - (G * cos(rollacc) .* cos(pitchacc)));

% Get the GPS data
xngps = fillin(time_base,data_gps(:,1),data_gps(:,3));
yngps = fillin(time_base,data_gps(:,1),data_gps(:,4));

% Get attitude KF data
roll = fillin(time_base,data_attitude(:,1),data_attitude(:,2));
pitch = fillin(time_base,data_attitude(:,1),data_attitude(:,3));
yaw = fillin(time_base,data_attitude(:,1),data_attitude(:,4));
rolld = fillin(time_base,data_attitude(:,1),data_attitude(:,5));
pitchd = fillin(time_base,data_attitude(:,1),data_attitude(:,6));
yawd = fillin(time_base,data_attitude(:,1),data_attitude(:,7));
rolldd = fillin(time_base,data_attitude(:,1),data_attitude(:,8));
pitchdd = fillin(time_base,data_attitude(:,1),data_attitude(:,9));
yawdd = fillin(time_base,data_attitude(:,1),data_attitude(:,10));

% Get position KF data
xn = fillin(time_base,data_position(:,1),data_position(:,2));
yn = fillin(time_base,data_position(:,1),data_position(:,3));
xbd = fillin(time_base,data_position(:,1),data_position(:,4));
ybd = fillin(time_base,data_position(:,1),data_position(:,5));
xbdd = fillin(time_base,data_position(:,1),data_position(:,6));
ybdd = fillin(time_base,data_position(:,1),data_position(:,7));

% Some plots
figure
scatter(yn,xn)
hold on
scatter(yngps,xngps)


figure
plot(F1)
hold on
plot(F2)
