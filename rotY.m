% rotY
%
% [R] = rotY(theta) Returns a rotation matrix describing a 
%                   rotation about the Y axis (theta in radians)
% 
% R = a 3x3 matrix describing the rotation of the object
% 
% theta = the angle of the rotaion in radians
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function R = rotY(theta)
  
cos_theta = cos(theta);
sin_theta = sin(theta);

R = [cos_theta, 0, sin_theta;
     0, 1, 0;
     -sin_theta, 0, cos_theta];
end

