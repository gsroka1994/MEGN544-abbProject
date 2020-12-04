% velocityJacobian
%
% [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
%     Retruns the forward kinematics of a manipulator with the provided DH
%     parameter set.
%
% Jv = the velocity jacobian
% JvDot = the time derivative of the velocity Jacobian
%
% linkList = an array of Link structures that have been created by
% CreateLink (see function for members)
% paramList = the array containing the current state of the joint variables
% according to the robot's encoders
% paramRateList = the current theta_dot and d_dot values for the joints. (an Nx1 array)
%
% Gunnar Sroka
% 10649539
% MEGN544
% 10/27/2020
function [Jv, JvDot] = velocityJacobian(linkList, paramList, paramRateList)
[m, n]= size(linkList);

T = eye(4,4);
z_list = zeros(3,n);
d_list = zeros(3,n);
w_list = zeros(3,n);
v_list = zeros(3,n);
w_total = zeros(3, 1);
v_total = zeros(3,1);
paramListIndex = 1;

for i = 1:n
    link = linkList(i);
    z_list(:, i) = T(1:3, 3);
    d_list(:, i) = T(1:3, 4);
    
    % Get our first set of variables that don't need T_This
    if link.isRotary == 1
        th = paramList(paramListIndex) - link.offset;
        d = link.d;
        if exist('paramRateList', 'var')
            w_total = w_total + paramRateList(paramListIndex)*z_list(:, i);
        end
    elseif link.isRotary == 0
        th = link.theta;
        d = paramList(paramListIndex) - link.offset;
        w_total = w_total + 0;
    else
        paramListIndex = paramListIndex - 1;
        th = link.theta;
        d = link.d;
        w_total = w_total + 0;
    end
    w_list(:, i) = w_total;
    v_list(:, i) = v_total;
    
    % Find T_This and then calculate the velocity
    T_this = dhTransform(link.a, d, link.alpha, th);
    x = T(1:3, 1:3) * T_this(1:3,4);
    if link.isRotary == 1
      v_total = v_total + cross(w_total, x);
    elseif link.isRotary == 0
      if exist('paramRateList', 'var')
          v_total = v_total + cross(w_total, x) + paramRateList(paramListIndex)*z_list(:, i); 
      end
    else
      paramListIndex = paramListIndex - 1;
      v_total = v_total + cross(w_total, x);  
    end
    
    % Update the Transform
    T = T*T_this;
    paramListIndex = paramListIndex + 1;
end

d0n = T(1:3, 4);
Jv = zeros(6, n);
JvDot = [];

for i = 1:n
    link = linkList(i);
    if link.isRotary == 1
        v = cross(z_list(:, i), (d0n - d_list(:, i)));
        Jv(:, i) = [v; z_list(:, i)];
    elseif link.isRotary == 0
        Jv(:, i) = [z_list(:, i); 0; 0; 0];
    end
end

if exist('paramRateList', 'var')
    JvDot = zeros(6, n);
    for i = 1:n
        link = linkList(i);
        a = cross(w_list(:, i), z_list(:, i));
        if link.isRotary == 1
            b = cross(a, d0n - d_list(:, i));
            c = b + cross(z_list(:, i), v_total - v_list(:, i));
            JvDot(:, i) = [c; a];
        elseif link.isRotary == 0
            JvDot(:, i) = [a; 0; 0; 0];
        end 
    end
end

end

