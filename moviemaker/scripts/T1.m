function [T1] = T1(alpha)
% This function provides the rotational matrix T1
%
% Input:   rotational degree in rad
% Output:  3x3-Rotational Matrix T1

T1 = [1 0 0; ...
      0 cos(alpha) sin(alpha); ...
      0 -sin(alpha) cos(alpha)];
end