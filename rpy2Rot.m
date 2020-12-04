% rpy2Rot
%
% [R] = rpy2Rot(roll, pitch, yaw) Returns a rotation matrix corresponding 
%                 to a roll, pitch, yaw encoded rotation.
%                 Note RPY is defined as the set of orthogonal 
%                 rotations rotZ(yaw)rotY(pitch)rotX(roll)
% 
% R = a 3x3 matrix describing the rotation of the object
% 
% roll = the angle of rotaion describing the objects roll
% pitch = the angle of rotaion describing the objects pitch
% yaw = the angle of rotaion describing the objects yaw
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function R = rpy2Rot(roll, pitch, yaw)
  R = rotZ(yaw) * rotY(pitch) * rotX(roll);
end

