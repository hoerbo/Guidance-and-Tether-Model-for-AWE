function [T2] = T2(alpha)
% This function provides the rotational matrix T2
%
% Input:   rotational degree in rad
% Output:  3x3-Rotational Matrix T2

T2 = [cos(alpha) 0 -sin(alpha); ...
      0 1 0; ...
      sin(alpha) 0 cos(alpha)];
end