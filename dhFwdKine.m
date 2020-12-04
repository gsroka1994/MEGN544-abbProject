% dhFwdKine
%
% H = dhFwdKine(linkList, paramList)
%     Retruns the forward kinematics of a manipulator with the provided DH
%     parameter set.
% 
% H = a Homogenous transform matrix
% 
% linkList = an array of Link structures that have been created by
% CreateLink (see function ofor members)
% paramList = the array containing the current state of the joint variables
% according to the robot's encoders
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 10/4/2020
function H = dhFwdKine(linkList, paramList)
  sz_ll = size(linkList);
  T = cell(sz_ll(2), 1);

  for i = 1:sz_ll(2)
     L = linkList(1, i);
     if L.isRotary == 1
         L.theta = paramList(i) - L.offset;
     elseif L.isRotary == 0
         L.d = paramList(i) - L.offset;
     end
     T{i, 1} = dhTransform(L.a, L.d, L.alpha, L.theta);
  end
  
  H = eye(4);
  for i = 1:sz_ll(2)
    H = H * T{i, 1};
  end
end

