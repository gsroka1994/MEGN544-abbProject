% dhInvKine
%
% [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
%     Returns the parameter list, according to the robot’s encoders, 
%     necessary to achieve a desired homogenous transform and the residual 
%     error in that transform. This should be achieved using Newton Raphson
%     or LM of the numerical approaches discussed in class
%
% paramList = returned list of params necessary to get to desTransform
% error = error between current guesss and desired
%
% linkList = an array of Link structures that have been created by
% createLink (see function for members)
% desTransform = the desired homogenous transform
% paramListGuess = an initial guess at the parameters, according to the 
% robot’s encoders. Possibly the current arm state?
%
% Gunnar Sroka
% 10649539
% MEGN544
% 10/27/2020
function [paramList, error] = dhInvKine(linkList, desTransform, paramListGuess)
%% Initialize our variables
p = paramListGuess;
Tc = dhFwdKine(linkList, p);
t_error = transError(desTransform, Tc);
dp = [inf; inf; inf; inf; inf; inf];
tolerance = sqrt(eps);

%% Use Gauss-Newton Root Method
while norm(t_error) > tolerance && norm(dp) > tolerance
   [Jv, JvDot] = velocityJacobian(linkList, p);
   [U, S, V] = svd(Jv);
   
   Jv_inv = V * pinv(S) * U';
   dp = Jv_inv * t_error;
   p = p + dp;
   Tc = dhFwdKine(linkList, p);
   t_error = transError(desTransform, Tc);
end

paramList = p;
error = norm(t_error);
end
