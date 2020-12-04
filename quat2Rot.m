% quat2Rot
%
%  R = quat2Rot(Q) Returns the rotation matrix that corresponds 
%                  to the quaternion, stacked [q0;q_vec].
%
% Q = the Quaternion describing the rotation
%
% R = a 3x3 matrix describing the rotation of the object
%
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function R = quat2Rot(Q)

  complex_vector = [Q(2); Q(3); Q(4)];
  I = eye(3,3);
  first_part = (Q(1)^2 - transpose(complex_vector)*complex_vector) * I;
  second_part = 2*Q(1)*cpMap(complex_vector);
  third_part = 2*complex_vector*transpose(complex_vector);
  
  R = first_part + second_part + third_part;
end

