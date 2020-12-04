% transform2Twist
%
% [t] = transform2Twist Returns the twist vector corresponding to 
%       the provided homogenous transform matrix. 
%       The twist should be stacked [v;w th].
% 
% t = twist vector
% 
% H =  The Homogenous transform matrix
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/8/2020

function t = transform2Twist(H)
 R = [H(1, 1:3); H(2, 1:3); H(3, 1:3)];
 d = H(1:3, 4);

 [k, theta] = rot2AngleAxis(R);
 omega = theta*k;

 if norm(omega) == 0
    v = d; 

 else
   I = eye(3,3);
   crossK = cpMap(k);
 
   v1 = (sin(theta)/(2*(1-cos(theta))))*I;
   v2 = ((2*(1 - cos(theta)) - theta*sin(theta))/(2*theta*(1 - cos(theta)))) * k * k';
   v3 = .5*crossK;
   v = (v1 + v2 - v3) * d; 
 end 
 
 t = [v; omega];
end

