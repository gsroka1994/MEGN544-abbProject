% abbInvKine
%
% [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)
%     Returns the joint angles required to reach the desired transformation
%     for the ABB arm. Make sure you handle joint limits, the rotation 
%     degeneracy, and unreachable states gracefully.
% 
% th1, th2, th3, th4, th5, th6 = the 6 joint angles, according to the
% robot's ecoders.  These are real scaler values if th_last is provided,
% and a 8x1 vector if th_last is not provided (one for each possible
% solution)
%
% reachable = is true if the transform can be achievd.  If false, it cannot
%
% T_des = desired homogeneous transform
%
% th_last = a 6x1 vector of the previous thetas used.  If not provided,
% return all possible solutions.  Otherwise, return closest solution to
% th_last (real values!!)
%
% Notes: The encoders are shifted relative to the DH-Zero Angle.  To
% convert: Theta_Arm = Theta_DH - Theta_Offset
% Only th2 has an offeset (pi/2), the rest are 0
%
% Gunnar Sroka
% 10649539
% MEGN544 
% 10/7/2020 (started, lol)
function [th1, th2, th3, th4, th5, th6, reachable] = abbInvKine(T_des, th_last)

%% Assign the input parameters
alpha_dh = [-pi/2, 0, -pi/2, pi/2, -pi/2, 0];
a_dh = [0, 0.270, 0.07, 0, 0, 0];
d_dh = [0.290, 0, 0, 0.302, 0, 0.072];

%% Start with Theta 1 by finding d_05
theta1 = zeros(1,8);
d06 = T_des(1:3, 4);
d05 = d06 - T_des(1:3, 3)*d_dh(6);
theta1(1:4) = atan2(d05(2), d05(1));
theta1(5:8) = atan2(d05(2), d05(1)) + pi;

% Do some angle wrapping thing that shows up in the homework solutions
theta1 = theta1 - (abs(theta1)>pi).*sign(theta1)*2*pi;

%% Next is Theta 3
% We need to first find d_14
r01_left = rotZ(theta1(1)) * rotX(alpha_dh(1));
r01_right = rotZ(theta1(5)) * rotX(alpha_dh(5));

d14 = zeros(3, 2);
d14(:, 1) = r01_left' * (d05 - [0; 0; d_dh(1)]);
d14(:, 2) = r01_right' * (d05 - [0; 0; d_dh(1)]);

% Need the intermediate angle for the right triangle made from a4(x) and
% d4(y)
int_angle = atan2(d_dh(4), a_dh(3));
hyp = sqrt(d_dh(4)^2 + a_dh(3)^2);

% Do a quick real check on theta3
tmp1 = isreal(sqrt(((a_dh(2) + hyp)^2 - norm(d14(:,1))^2)/(norm(d14(:,1))^2 - (a_dh(2) - hyp)^2)));
tmp2 = isreal(sqrt(((a_dh(2) + hyp)^2 - norm(d14(:,2))^2)/(norm(d14(:,2))^2 - (a_dh(2) - hyp)^2)));
th3Real = 1;
if tmp1 == 0 || tmp2 == 0
    th3Real = 0;
end

% Now calculate theta 3 with manipulator equation.  Need to subtract the
% intermediate angle from the returned value of theta 3
theta3 = zeros(1,8);
theta3(1:4) = [1;1;-1;-1]*real(2*atan(sqrt(((a_dh(2) + hyp)^2 - norm(d14(:,1))^2)/(norm(d14(:,1))^2 - (a_dh(2) - hyp)^2))));
theta3(5:8) = [1;1;-1;-1]*real(2*atan(sqrt(((a_dh(2) + hyp)^2 - norm(d14(:,2))^2)/(norm(d14(:,2))^2 - (a_dh(2) - hyp)^2))));
theta3 = theta3 - int_angle;

% top_left = 2*a_dh(2)*hyp - a_dh(2)^2 - d_dh(4)^2 - a_dh(3)^2 + norm(d14(:,1))^2;
% top_right = 2*a_dh(2)*hyp - a_dh(2)^2 - d_dh(4)^2 - a_dh(3)^2 + norm(d14(:,2))^2;
% 
% bottom_left = 2*a_dh(2)*hyp - a_dh(2)^2 - d_dh(4)^2 - a_dh(3)^2 + norm(d14(:,1))^2;
% bottom_right = 2*a_dh(2)*hyp + a_dh(2)^2 + d_dh(4)^2 + a_dh(3)^2 - norm(d14(:,2))^2;
% 
% theta3(1:4) = [1;1;-1;-1]*real(2*atan(sqrt(top_left/bottom_left)));
% theta3(5:8) = [1;1;-1;-1]*real(2*atan(sqrt(top_right/bottom_right)));
% theta3 = pi + theta3 - int_angle;

theta3 = theta3 - (abs(theta3)>pi).*sign(theta3)*2*pi;

%% Theta 2
theta2 = zeros(1, 8);
gamma = zeros(1,8);
gamma(1:8) = atan2(hyp*sin(theta3(1:8) + int_angle), a_dh(2) + hyp*cos(theta3(1:8) + int_angle));
theta2(1:4) = -gamma(1:4) + atan2(d14(2,1), d14(1,1)) + pi/2;
theta2(5:8) = -gamma(5:8) + atan2(d14(2,2), d14(1,2)) + pi/2;

% alpha = (a_dh(2) + a_dh(3)*cos(theta3(1:8)) + d_dh(4)*cos(theta3(1:8) + pi/2));
% beta = (a_dh(3)*sin(theta3(1:8)) + d_dh(4)*sin(theta3(1:8) + pi/2));
% theta2(1:4) = atan2((-alpha(1:4)*d14(3,1) - beta(1:4)*d14(1,1))/(alpha(1:4).^2 + beta(1:4).^2), (alpha(1:4)*d14(1,1) - beta(1:4)*d14(3,1))/(alpha(1:4).^2 + beta(1:4).^2));
% theta2(5:8) = atan2((-alpha(5:8)*d14(3,1) - beta(5:8)*d14(1,1))/(alpha(5:8).^2 + beta(5:8).^2), (alpha(5:8)*d14(1,1) + beta(5:8)*d14(3,1))/(alpha(1:4).^2 + beta(5:8).^2));
theta2 = theta2 - (abs(theta2)>pi).*sign(theta2)*2*pi;

%% Theta 5 => 4,6
% Now that we have Theta's 1-3 we can make some sweet, sweet transforms to
% figure out O4, and O5
theta5 = zeros(1,8);
theta4 = zeros(1,8);
theta6 = zeros(1,8);
r_later = cell(1,8);

for i = 1:2:8
    T03 = dhTransform(a_dh(1), d_dh(1), alpha_dh(1), theta1(i)) * dhTransform(a_dh(2), d_dh(2), alpha_dh(2), theta2(i)-pi/2) * dhTransform(a_dh(3), d_dh(3), alpha_dh(3), theta3(i));
    r03 = T03(1:3, 1:3);
    r36 = r03' * T_des(1:3, 1:3);
    r_later{1,i} = r36;
    r_later{1,i+1} = r36;
    
    % From Forward kinematics R(3,3) = cos(theta5), so use R(1,3) and R(2,3)
    % to solve
    theta5(1, i) = atan2(sqrt(r36(3,1)^2 + r36(3,2)^2), r36(3,3));
    theta5(1, i+1) = atan2(-sqrt(r36(3,1)^2 + r36(3,2)^2), r36(3,3));
    theta5(i) = theta5(i) - (abs(theta5(i))>pi).*sign(theta5(i))*2*pi;
    theta5(i+1) = theta5(i+1) - (abs(theta5(i+1))>pi).*sign(theta5(i+1))*2*pi;
    
    % Moving on to theta4 and theta6, a degeneracy exists when theta5 is pi
    % or 0.  If 0, same degeneracy that appears in ZYZ
    if abs(sin(theta5(1, i))) < 10*eps*128 || abs(sin(theta5(1, i+1))) < 10*eps*128
        theta4(1, i) = 0;
        theta4(1, i+1) = 0;
        theta6(1, i) = atan2(r36(2,1), r36(1,1));
        theta6(1, i+1) = atan2(r36(2,1), r36(1,1));
    % If not, solve this problem like its a ZYZ solution (as per lecture
    % slides on spherical wrist and Hollerbach Course Notes
    else
        theta4(1, i) = atan2(-r36(2,3)/sin(theta5(1,i)), -r36(1,3)/sin(theta5(1,i)));
        theta4(1, i+1) = atan2(-r36(2,3)/sin(theta5(1,i+1)), -r36(1,3)/sin(theta5(1,i+1)));
        theta6(1, i) = atan2(-r36(3,2)/sin(theta5(1,i)), r36(3,1)/sin(theta5(1,i)));
        theta6(1, i+1) = atan2(-r36(3,2)/sin(theta5(1,i+1)), r36(3,1)/sin(theta5(1,i+1)));
    end
end

%% Check for previous values of theta so that we can choose the correct solution
if size(th_last) > 0
    
    % Address the special cases for theta5
    % Use least square solution for th4, th6 in this scenario
    for i = 1:8
        if abs(sin(theta5(i))) < sqrt(eps) % If theta is 0 or pi (or -pi or 2pi or -2pi)
            if round(cos(theta5(i)), 3) == 1 % If theta is not pi or -pi
                tmp5 = [theta5(i); theta5(i) + 2*pi];  % Theta5 could be 0 or 2*pi.  Pick closest
                [~, ind] = min((tmp5-th_last(5)).^2);
                theta5(i) = tmp5(ind);
                
                % Do least squares method (a -pi option).  Pick closest
                tmp4 =  [0.5*th_last(4) - 0.5*th_last(6) + 0.5*theta6(i); 0.5*th_last(4) - 0.5*th_last(6) + 0.5*theta6(i) - pi];
                tmp6 = [-0.5*th_last(4) + 0.5*th_last(6) + 0.5*theta6(i); -0.5*th_last(4) + 0.5*th_last(6) + 0.5*theta6(i) - pi];
                
                [~, ind] = min((tmp4-th_last(4)).^2);
                theta4(i) = tmp4(ind);
                
                [~, ind] = min((tmp6-th_last(6)).^2);
                theta6(i) = tmp6(ind);
                
            else % Theta is pi or -pi.  Least squares but neead to recalculate value of theta6 to use as last entry
                theta4(i) = 0.5*th_last(4) + 0.5*th_last(6) - 0.5*atan2(r_later{1,i}(2,1),-r_later{1,i}(1,1));
                theta6(i) = 0.5*th_last(4) + 0.5*th_last(6) + 0.5*atan2(r_later{1,i}(2,1),-r_later{1,i}(1,1)); 
            end
        end
    end
    theta4 = theta4 - (abs(theta4)>pi).*sign(theta4)*2*pi;
    theta6 = theta6 - (abs(theta6)>pi).*sign(theta6)*2*pi;
 
    % For figuring out the true distance between the angles, we need to
    % 'unwrap the angles'
    % ex. theta4 = -3.10 and th_last = 3.10.  The real distance between
    % those two angle is .08
    min1 = theta1 - (abs(theta1-th_last(1))>pi).*sign(theta1-th_last(1))*2*pi;
    min2 = theta2 - (abs(theta2-th_last(2))>pi).*sign(theta2-th_last(2))*2*pi;
    min3 = theta3 - (abs(theta3-th_last(3))>pi).*sign(theta3-th_last(3))*2*pi;
    min4 = theta4 - (abs(theta4-th_last(4))>pi).*sign(theta4-th_last(4))*2*pi;
    min5 = theta5 - (abs(theta5-th_last(5))>pi).*sign(theta5-th_last(5))*2*pi;
    min6 = theta6 - (abs(theta6-th_last(6))>pi).*sign(theta6-th_last(6))*2*pi;
   
    % Choose the solution set that has the overall min distance
    [~,ind] = min((min1-th_last(1)).^2 + ...
        (min2-th_last(2)).^2 + ...
        (min3 -th_last(3)).^2 + ...
        (min4 - th_last(4)).^2+...
        (min5-th_last(5)).^2 +...
        (min6-th_last(6)).^2);
    th1 = theta1(ind);
    th2 = theta2(ind);
    th3 = theta3(ind);
    th5 = theta5(ind);
    th4 = theta4(ind);
    th6 = theta6(ind);
    
else
    th1 = theta1';
    th2 = theta2';
    th3 = theta3';
    th4 = theta4';
    th5 = theta5';
    th6 = theta6';
end

%% Finally, check if any of this crap is reachable by the robotic arm
if isreal(th1) && isreal(th2) && isreal(th3) && isreal(th4) && isreal(th5) && isreal(th6) && th3Real
    reachable = 1;
else
    reachable = 0;
end

end