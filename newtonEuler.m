% newtonEuler
%
% [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundary_conditions)
%     Computes the inverse dynamics of a serial link manipulator
% 
% linkList – a list of the joint parameters created by createLink
% paramList – the current joint angles/distances. (an Nx1 array)
% paramListDot – the current joint angle/distance speeds. (an Nx1 array)
% paramListDDot – the current joint angle/distance accelerations. (an Nx1 array)
% boundry_conditions – a structure containing:
%                      base_angular_velocity
%                      base_angular_acceleration
%                      base_linear_acceleration (add gravity in here)
%                      distal_force (in tool frame)
%                      distal_torque (in tool frame)
%
% Gunnar Sroka
% 10649539
% MEGN544 
% 11/13/2020
function [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList, paramListDot, paramListDDot, boundary_conditions)
[m, n]= size(linkList);

%% PreAllocate the variables
Jv = zeros(6, n);
JvDot = zeros(6, n);
jointTorques = zeros(n, 1);

T = eye(4,4);
z_list = zeros(3,n);
d_list = zeros(3,n);
w_list = zeros(3,n);

if_list = zeros(3,n);
ii_list = zeros(3,n);
T_list = zeros(4,4,n);

w_base = boundary_conditions.base_angular_velocity;
w_total = w_base;
a_total = boundary_conditions.base_linear_acceleration;
al_total = boundary_conditions.base_angular_acceleration;

rdd_total = zeros(3,1);

%% Calculate the Kinetic Values
paramListIndex = 1;
for i = 1:n
    
    % Setup the z and d lists from the transform
    link = linkList(i);
    z_list(:, i) = T(1:3, 3);
    d_list(:, i) = T(1:3, 4);
    
    % Get our transform for the current link
    if link.isRotary == 1
        th = paramList(paramListIndex) - link.offset;
        d = link.d;
    elseif link.isRotary == 0
        th = link.theta;
        d = paramList(paramListIndex) - link.offset;
    else
        paramListIndex = paramListIndex - 1;
        th = link.theta;
        d = link.d;
    end
    
    % Find T_This and then calculate the linear and COM accelerations
    T_this = dhTransform(link.a, d, link.alpha, th);
    
    % Update the Transform
    T = T*T_this;
    
    x = T(1:3, 4) - d_list(:,i);
    rcom = T(1:3, 1:3) * (T_this(1:3, 1:3)'*T_this(1:3,4) + link.com);
    
    % Get the angular and linear values
    % Using equations from Hollerbach Course Notes
    if link.isRotary == 1
        al_total = al_total + paramListDDot(paramListIndex)*z_list(:, i) + paramListDot(paramListIndex)*cross(w_total, z_list(:, i));
        w_total = w_total + paramListDot(paramListIndex)*z_list(:, i);
        rdd_total = a_total + cross(w_total, cross(w_total, rcom)) + cross(al_total, rcom);
        a_total = a_total + cross(w_total, cross(w_total, x)) + cross(al_total, x);
    elseif link.isRotary == 0
        rdd_total = a_total + cross(w_total, cross(w_total, rcom)) + cross(al_total, rcom);
        a_total = a_total + cross(al_total, x) + cross(w_total, cross(w_total, x));
        
        extra = 2*paramListDot(paramListIndex)*cross(w_total, z_list(:, i)) + paramListDDot(paramListIndex)*z_list(:,i);
        
        a_total = a_total + extra;
        rdd_total = rdd_total + extra;
    end
    
    w_list(:, i) = w_total - w_base;
    T_list(:,:,i) = T_this;
    
    % Calculate link forces and torques
    if_list(:,i) = link.mass * (T(1:3,1:3)'*rdd_total);
    ii_list(:,i) = link.inertia*T(1:3,1:3)'*al_total + cross(T(1:3,1:3)'*w_total, link.inertia*T(1:3,1:3)'*w_total);
    
    paramListIndex = paramListIndex + 1;
end

%% Now calculate all the Forces and Torques
F_list = zeros(3,n);
Tor_list = zeros(3,n);

F_total = boundary_conditions.distal_force;
Tor_total = boundary_conditions.distal_torque;
Rii = eye(3);

i = n;
while i > 0
    if i == n
        Rii = eye(3);
    else
        Rii = T_list(1:3,1:3,i+1);
    end
    
    link = linkList(i);
    R = T_list(1:3,1:3,i);
    F_list(:, i) = if_list(:, i) + Rii*F_total;
    F_total = F_list(:, i);
    
    D = T_list(1:3,4,i);
    Tor_list(:, i) = ii_list(:, i) + Rii*Tor_total + cross(R'*D, F_list(:,i)) + cross(link.com, if_list(:,i));
    Tor_total = Tor_list(:, i);
    
    if linkList(i).isRotary == 0
        jointTorques(i,1) = dot([0;0;1], R * F_list(:,i));
    else
        jointTorques(i,1) = dot([0;0;1], R * Tor_list(:,i));
    end
    
    i = i - 1;
end

% Jacobian Time
[Jv, JvDot] = velocityJacobian(linkList, paramList, paramListDot);

end