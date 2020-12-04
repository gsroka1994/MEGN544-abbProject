% rotationError
%
% rot_error_vector = rotationError(Rot_desired, Rot_current) 
% Returns an angle*axis vector, expressed in the reference frame, 
% describing what rotation is necessary to take Rot_current to Rot_desired
% 
% rot_error_vector = a 3x1 vector describing the axis of rotation 
% multiplied by the angle of rotation (in radians) necessary to 
% transform Rot_current into Rot_desired
%
% Rot_desired = rotation matrix describing the desired coordinate frame
% Rot_current = rotation matrix describing the current coordinate frame
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 10/27/2020

function rot_error_vector = rotationError(Rot_desired, Rot_current)
  rot_error = Rot_current' * Rot_desired;
  [k, theta] = rot2AngleAxis(rot_error);
  
  rot_error_vector = Rot_current* (theta * k);
end

