function [guidance,tethermodel,NDI_Mode] = settings()

%% Kinematics
guidance.vk = single([60;0;0]); %commanded kinematic velocity

%% Guidance
guidance.alpha = single(90/180*pi); %rotation angle of lemniscate
guidance.epsilon = single(45/180*pi); %elevation angle of lemniscate
guidance.ap = single(120); %half (flown) lemniscate width in m
guidance.windangle = single(0); %constant wind from north
guidance.vkr_norm = single(8); %reel-out speed in m/s
guidance.ak_r = single(0); %reel-out acceleration in m/s^2
guidance.delta0 = single(0.1); %turning point distance (gain in empirical law)
guidance.kdelta = single(9); %gain for NDI
guidance.heightcmd = single(350); %target height (starting orbit for vkr_norm =! 0)

%Choose between Control Law based on NDI or an Empirical Law for the FB
%Course and Rate:
NDI_Mode = 1; %Mode=1: NDI, Mode=0: Empirical Law

%% Tether Model
tethermodel.pm = 6; %number of pointmasses
tethermodel.L_0tot = 300; %initial tether length
tethermodel.L0 = tethermodel.L_0tot/tethermodel.pm; %initial segment length
tethermodel.timestep = 0.01;
tethermodel.kp = 0.7; %winch controller p-gain
tethermodel.rw = 0.1; %winch diameter

%Calculate initial conditions for simulation time = 0
g = [0;0;9.81];
Y0hat = zeros(3,2*tethermodel.pm);
Yp0hat = zeros(3,tethermodel.pm);
for i=1:tethermodel.pm
    Y0hat(:,i) = [0;0;-tethermodel.L0*(i-1)];
    Y0hat(:,i+tethermodel.pm) = -guidance.vk'*(norm(Y0hat(:,i))/tethermodel.L_0tot);
    Yp0hat(:,i) = g;
    
end
Y0hat = reshape(Y0hat,[6*tethermodel.pm,1]);
Yp0hat = reshape(Yp0hat,[3*tethermodel.pm,1]);

tethermodel.Y0 = Y0hat;
tethermodel.Yp0 = [zeros(tethermodel.pm*3,1);Yp0hat];
end