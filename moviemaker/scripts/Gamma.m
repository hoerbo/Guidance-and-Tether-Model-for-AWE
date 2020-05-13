function x   = Gamma(sstar, guidance)
% This function provides the target trajectory Gamma parametrized by the
% path parameter s
% Hint: Function is adapted to accept Matrix Inputs (pointwise operation)
%
% Input:   path parameter s
%          rotation angle of the lemniscate alpha
%          elevation angle of the lemniscate epsilon
%          commanded lemniscate width in flight ap
% Output:  target trajectory

alpha = guidance.alpha;
epsilon = guidance.epsilon;
a = min(guidance.ap/guidance.heightcmd,1); %calculate a based on commanded ap and height

%Analytically derived equations of the lemniscate in spherical coordinates 
%and zenith position
lambda = atan(sin(sstar));
phi = acos((a.*cos(sstar))./(1+sin(sstar).^2));

%Transformation in cartesian coordinates and rotation/elevation
x = T2(pi/2-epsilon)'*T3(alpha)'*s2c(lambda,phi);
end