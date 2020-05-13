function [T3] = T3(alpha)
% This function provides the rotational matrix T3
%
% Input:   rotational degree in rad
% Output:  3x3-Rotational Matrix T3

T3 = [cos(alpha) sin(alpha) 0; ...
      -sin(alpha) cos(alpha) 0; ...
      0 0 1];
end