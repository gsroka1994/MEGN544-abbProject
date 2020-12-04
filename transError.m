% transError
%
% error_vector = rotationError(Rot_desired, Rot_current) 
% Returns an 6x1 vector, where the first 3 elements are position error 
% (desired - current), and the last three elements are an angle-axis 
% representation of rotation error. Both expressed in the shared base frame.
% 
% error_vector = a 6x1 vector describing the error ([pos_error;rot_error])
% as expressed in the shared base frame
%
% Td = homogeneous matrix describing the desired coordinate frame
% Tc = homogeneous matrix describing the current coordinate frame
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 10/27/2020

function error_vector = transError(Td, Tc)
  rot_error = rotationError(Td(1:3, 1:3), Tc(1:3, 1:3));
  d_error = Td(1:3,4) - Tc(1:3, 4);
  
  error_vector = [d_error; rot_error];
end

