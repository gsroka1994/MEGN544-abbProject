% twist2Transform
%
% [H] = twist2Transform Returns the homogenous transformation matrix 
%       corresponding to a 6 element twist vector. 
%       The twist should be stacked [v;w th]
% 
% H =  The Homogenous transform matrix
% 
% t = twist vector
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/8/2020

function H = twist2Transform(t)
  v = t(1:3);
  omega = t(4:6);
  
  theta = norm(omega);
  k = omega/theta;
  crossK = cpMap(k);
  
  sin_theta = sin(theta);
  cos_theta = cos(theta);
  
  I = eye(3,3);
  R = cos_theta*I + (1-cos_theta) * k * transpose(k) + sin_theta*crossK;
  d = ((I - R)*crossK + omega*k') * v;
  
  H = [R, d; 0, 0, 0, 1];
end

