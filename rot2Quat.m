% rot2Quat
%
% [Q] = rot2Quat(R) Returns the quaternion [qo;q_vec] 
%        that corresponds to the rotation matrix.
%
% R = a 3x3 matrix describing the rotation of the object
%
% Q = the Quaternion describing the rotation
%
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function Q = rot2Quat(R)
  check_alt_q0 = (1 + trace(R))/4;
  
  q0 = sqrt(check_alt_q0);
  q0_neg = -q0;
  
  q1 = (R(3,2) - R(2,3))/(4*q0);
  q1_neg = -q1;
  
  q2 = (R(1,3) - R(3,1))/(4*q0);
  q2_neg = -q1;
  
  q3 = (R(2,1) - R(1,2))/(4*q0);
  q3_neg = -q1;
  
  complex_vec = [q1, q2, q3];
  
  Q = [q0; transpose(complex_vec)];

end

