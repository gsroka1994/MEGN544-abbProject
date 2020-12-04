% rotZ
%
% [R] = rotZ(theta) Returns a rotation matrix describing a 
%                   rotation about the Z axis (theta in radians)
% 
% R = a 3x3 matrix describing the rotation of the object
% 
% theta = the angle of the rotaion in radians
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function R = rotZ(theta)
  
cos_theta = cos(theta);
sin_theta = sin(theta);

R = [cos_theta, -sin_theta, 0;
    sin_theta, cos_theta, 0
    0, 0, 1];
end

