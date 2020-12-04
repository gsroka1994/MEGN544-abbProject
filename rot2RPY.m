% rot2RPY
%
% [roll, pitch, yaw] = Returns the roll, pitch and yaw corresponding to a given rotation matrix. 
% It should return the two valid solutions corresponding to the +sqrt and –sqrt. 
% Each output is then a [2x1] vector with the plus solution on top
% 
% roll = the angle of rotaion describing the objects roll
% pitch = the angle of rotaion describing the objects pitch
% yaw = the angle of rotaion describing the objects yaw
%
% R = a 3x3 matrix describing the rotation of the object
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function [roll, pitch, yaw] = rot2RPY(R)

  pitch_plus_sqrt = atan2(-R(3,1), sqrt(R(1,1)^2 + R(2,1)^2));
  pitch_neg_sqrt = atan2(-R(3,1), -sqrt(R(1,1)^2 + R(2,1)^2));
  
  if pitch_plus_sqrt == pi/2
    yaw_plus_sqrt = 0;
    roll_plus_sqrt = atan2(R(1,2), R(2,2));
    
    yaw_neg_sqrt = 0;
    roll_neg_sqrt = atan2(R(1,2), R(2,2));
    
  elseif pitch_neg_sqrt == -pi/2
    yaw_plus_sqrt = 0;
    roll_plus_sqrt = -atan2(R(1,2), R(2,2));
    
    yaw_neg_sqrt = 0;
    roll_neg_sqrt = -atan2(R(1,2), R(2,2));
    
  else
      yaw_plus_sqrt = atan2(R(2,1)/cos(pitch_plus_sqrt), R(1,1)/cos(pitch_plus_sqrt));
      roll_plus_sqrt = atan2(R(3,2)/cos(pitch_plus_sqrt), R(3,3)/cos(pitch_plus_sqrt));
      
      yaw_neg_sqrt = atan2(R(2,1)/cos(pitch_neg_sqrt), R(1,1)/cos(pitch_neg_sqrt));
      roll_neg_sqrt = atan2(R(3,2)/cos(pitch_neg_sqrt), R(3,3)/cos(pitch_neg_sqrt));
  end
    
  roll = [roll_plus_sqrt; roll_neg_sqrt];
  pitch = [pitch_plus_sqrt; pitch_neg_sqrt];
  yaw = [yaw_plus_sqrt; yaw_neg_sqrt];
 

end


