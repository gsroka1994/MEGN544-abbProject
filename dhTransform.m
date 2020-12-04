% dhTransform
%
% H = dhTransform(a, d, alpha, theta) Returns the chained transform 
%     that adheres to the passed in DH parameters.
% 
% H = a Homogenous transform matrix
% 
% a = the distance in the X direction
% d = the distance in the Z direction
% alpha = the angle in the X direction
% theta = the angle in the Z direction
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/15/2020
function H = dhTransform(a, d, alpha, theta)
  trans_x = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
  trans_z = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d; 0, 0, 0, 1];
  
  rot_x = rotX(alpha);
  rot_z = rotZ(theta);
  
  H_r_x = [rot_x, [0;0;0]; 0, 0, 0, 1];
  H_r_z = [rot_z, [0;0;0]; 0, 0, 0, 1];
  
  H = trans_z * H_r_z * trans_x * H_r_x;

end

