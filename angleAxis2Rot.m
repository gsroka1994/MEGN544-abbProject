% angleAxis2Rot
%
% [R] = angleAxis2Rot(k, theta) Returns the rotation matrix encoded 
%               by a rotation of theta radians about the unit vector k axis.
% 
% R = a 3x3 matrix describing the rotation of the object
% 
% k = unit vector that describes axis of rotation
% theta = the angle of rotaion
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function R = angleAxis2Rot(k, theta)
  if norm(k) < sqrt(eps)
    R = eye(3);
    return;
  elseif abs(norm(k)-1)>sqrt(eps)
    theta = theta+norm(k);
    k = k/norm(k);
  end

R = cos(theta)*eye(3) + sin(theta)*cpMap(k) + (1-cos(theta))*(k*k');
end

