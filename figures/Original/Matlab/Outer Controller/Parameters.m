% Parameters
m=13;     % [kg] mass
mx=m;
my=m;
Ix=0.06541;     % [kgm2]
Iy=1.08921;     % [kgm2]
Iz=1.10675;     % [kgm2]

dx=2.86;    % [] 
dy=32.5;	% []
dz=0;    % []
droll=0.1094;  % []
dpitch=7.203; % []
dyaw=0.22285;    % []

l1=0.05;   % [m]
l2=0.05;   % [m]

Troll=6.9736;   % [N/m]
Tpitch=131.8316;   % [N/m]

Ts=0.05;
% Path controller parameters
r=2.5;
R=r;
AcceptRadius=1;

wps = [1 0;
       1 50];