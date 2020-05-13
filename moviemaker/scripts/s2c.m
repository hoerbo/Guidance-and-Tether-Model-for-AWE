function [x] = s2c(lambda,phi)
% This function provides a transformation from spherical coordinates to
% cartesian coordinates.
%
% Input:    longitude = lambda
%           latitude = phi
% Output:   x = cartesian vector in 3xn

x = [cos(lambda).*cos(phi);...
     sin(lambda).*cos(phi);...
     sin(phi)];
end