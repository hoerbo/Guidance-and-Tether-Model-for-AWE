function [Y,Tetherforce] = tether_model(Xg,Vk,Y,pm,L0,timestep)
%This function provides initial conditions and residuals for interrupted
%solving of the implicit ODE
%Attention! ODE15i only accepts double data types/does not accept a mix of data types

%Basic Properties for Dyneema SK78 1mm Tether
E = 109*1e9; %N/m^2 (109 GPa)
dt = 1e-3; %m
rho_dyn = 970; %kg/m^3
A = pi*dt^2/4; %m^2
k = E*A/L0; %N/m (local spring force)
c = 60 / L0; %Nm/s: (Linskens et al.: Tether Dynamics Analysis...: 0.3*200m)
mass = rho_dyn * A * L0 * pm / (pm-1); %kg
cd = 0.958; 

%Calculate Initial Conditions in the Beginning of every time step
[Y0,Yp0,Tetherforce] = initialcondition(Y,Xg,Vk,pm,L0,k,c,cd,mass,dt);

%Solve ODE
[~,Y] = ode15i(@residual,[0 timestep],Y0,Yp0,[],Xg,Vk,pm,L0,k,c,cd,mass,dt);
Y = Y(end,:)';
end

function [Y0,Yp0,Tetherforce] = initialcondition(Y,Xg,Vk,pm,L0,k,c,cd,mass,dt)
%this function calculates the initial condition between time steps

%constants
g = [0;0;9.81];
rho = 1.225;

Y0 = Y;

%To Do: implement local wind at every point mass
vw=zeros(3,pm);
for i=1:pm
    vw(:,i) = [0;0;0];
end

%reshape state vector in particle form
Yhat = reshape(Y,[3,pm*2]);
p = Yhat(:,1:pm);
v = Yhat(:,pm+1:end);

%velocity
Yp0 = zeros(3,2*pm); %instantitate Yp0 in particle form
for i=1:pm
    Yp0(:,i) = v(:,i);
end

%acceleration
% calculate total forces on point masses
totalforce = zeros(3,pm);
for i=1:pm-1 %iterate through segments
    [p1force,p2force] = pointforce(p(:,i),p(:,i+1),v(:,i),v(:,i+1),vw(:,i),vw(:,i+1),cd,rho,dt,k,c,L0);
    totalforce(:,i) = totalforce(:,i) + p1force;
    totalforce(:,i+1) = totalforce(:,i+1) + p2force;
end

%Calculate Tether Force (vw at plane = vw at last point mass)
[p1force,Tetherforce] = pointforce(p(:,pm),Xg,v(:,pm),Vk,vw(:,pm),vw(:,pm),cd,rho,dt,k,c,L0);
totalforce(:,pm) = totalforce(:,pm) + p1force;

for i=1:pm
    Yp0(:,i+pm) = totalforce(:,i)/mass + g;
end

%% Reshape to State Vector Form
Yp0 = reshape(Yp0,[pm*6,1]);
end

function res = residual(t,Y,Yp,Xg,Vk,pm,L0,k,c,cd,mass,dt)

%constants
g = [0;0;9.81];
rho = 1.225;

%reshape state vectors in particle form
Yhat = reshape(Y,[3,pm*2]);
p = Yhat(:,1:pm);
v = Yhat(:,pm+1:end);

Yphat = reshape(Yp,[3,pm*2]);
dp = Yphat(:,1:pm);
dv = Yphat(:,pm+1:end);

%To Do: implement local wind at every point mass
vw=zeros(3,pm);
for i=1:pm
    vw(:,i) = [0;0;0];
end

%% Rp

Rp = zeros(3,pm); %instantitate Rp Vector in particle form
Rp(:,1) = p(:,1); %first set is position of particle 1
for i=2:pm
    Rp(:,i) = v(:,i) - dp(:,i);
end

%% Rv

Rv = zeros(3,pm); %instantitate Rp Vector in particle form
Rv(:,1) = v(:,1); %first set is velocity of particle 1

% calculate total forces on point masses
totalforce = zeros(3,pm);
for i=1:pm-1 %iterate through segments
    [p1force,p2force] = pointforce(p(:,i),p(:,i+1),v(:,i),v(:,i+1),vw(:,i),vw(:,i+1),cd,rho,dt,k,c,L0);
    totalforce(:,i) = totalforce(:,i) + p1force;
    totalforce(:,i+1) = totalforce(:,i+1) + p2force;
end

%Calculate Tether Force (vw at plane approximated)
[p1force,~] = pointforce(p(:,pm),Xg,v(:,pm),Vk,vw(:,pm),vw(:,pm),cd,rho,dt,k,c,L0);
totalforce(:,pm) = totalforce(:,pm) + p1force;

for i=2:pm
    Rv(:,i) = dv(:,i) - totalforce(:,i)/mass - g;
end

%% Reshape to State Vector Form
Rp = reshape(Rp,[pm*3,1]);
Rv = reshape(Rv,[pm*3,1]);
res = [Rp;Rv];

end

function [p1force,p2force] = pointforce(p1,p2,v1,v2,vw1,vw2,cd,rho,dt,k,c,L0)
%This function calculates the forces induced by one segment and allocates
%them to the neighbouring point masses

%% Dragforce

%average segment speed
dSa = (v1+v2)/2;

%average wind speed at segment
vwa = (vw1+vw2)/2;

%aparent air speed at segment
ava = vwa-dSa;

%segment vector
S = p2-p1;

%norm of segment vector
S_norm = norm(S);

%segment unit vector
e_S = S/S_norm; 

%aparent air speed perpendicular to the segment
ava_perp = ava - ava'*e_S*e_S;

%area
A = dt*S_norm;

%drag force (rho=const.)
dragforce = 0.5*rho*cd*norm(ava_perp)*ava_perp*A;

%% Spring and Damping Force

%segment relative speed
dS = v2-v1;

%scalar spring force
sf = k*(S_norm - L0); %tension

if S_norm < L0 %compression
    sf = 0.1*sf; 
end

%scalar damping force (constant for tension and compression)
df = c*(dS'*e_S);

%segment spring and damping force
springdampingforce = (sf + df)*e_S;

%% Summation

p1force = dragforce/2 + springdampingforce;
p2force = dragforce/2 - springdampingforce;
end