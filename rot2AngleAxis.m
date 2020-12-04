% rot2AngleAxis
%
% [k, theta] = rot2AngleAxis(R) Returns the rotation matrix encoded by a 
% rotation of theta radians about the unit vector k axis.
% 
% k = unit vector that describes axis of rotation
% theta = the angle of rotaion
%
% R = a 3x3 matrix describing the rotation of the object
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function [k, theta] = rot2AngleAxis(R)
  
 mat_aa = [R(3,2) - R(2,3);
           R(1,3) - R(3,1);
           R(2,1) - R(1,2)];
       
theta = atan2(.5 * norm(mat_aa), .5*(trace(R)-1));

if abs(theta-pi) <= eps(2) || abs(theta-0) <= eps(2)
      kx = sqrt((R(1,1) + 1)/2);
      ky_p = (R(1,2) + R(2,1))/(4*kx);
      kz_p = (R(1,3) + R(3,1))/(4*kx);
      
      ky_n = (R(1,2) + R(2,1))/(4*-kx);
      kz_n = (R(1,3) + R(3,1))/(4*-kx);
      
      k = [kx; ky_p; kz_p];
else
   k = (1/(2*sin(theta))) * mat_aa; 
end

end

