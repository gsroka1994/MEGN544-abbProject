% constAccelInterp
%
% [p, v, a] = constAccelInterp(t, trajectory, transPercent)
% Provides the position, velocity, and acceleration at a time t for a
% trajectory interpolated using the constant acceleration approach.  Each
% of these are length M vectors
%
% p, v, a = position, velocity, acceleration (length M)
%
% t = the time of interest
% trajectory = a Nx(M+1) array of points.  There are N timestamps.  Column
% one is time, with the remaining columns providing the point
% transPercent = the percentage of the trajectory time to use for the
% constant acceleration transition.  Must be between [0, 0.5].  It is to be
% taken relative to the shorter duration segment when interpolating between
% two linear segments
%
% Gunnar Sroka
% 10649539
% MEGN544
% 10/4/2020
function [p, v, a] = constAccelInterp(t, trajectory, transPercent)

% Preallocate our vectors
[n,m] = size(trajectory);
p = zeros(1,m-1);
v = zeros(1, m-1);
a = zeros(1, m-1);

% Get the tau's
tau_list = zeros(1,n);
for i=1:n-1
    tau_list(i)=transPercent*(trajectory(i+1, 1)-trajectory(i, 1));
end
tau_list(n)=tau_list(n-1);

for j = 2:n-1 % Time Slices
    % Calculate values that we'll need no matter the scenario
    t1 = trajectory(j, 1) - tau_list(j);
    A = trajectory(j-1, 2:end);
    B = trajectory(j, 2:end);
    t_a = trajectory(j - 1, 1);
    t_b = trajectory(j, 1);
    v_ab = (B-A)/(t_b - t_a);
    
    % Const Velocity Segment: Check if t is before the curve leading
    % into the current Point
    t0 = trajectory(j - 1, 1) + tau_list(j - 1);
    if t0 < t && t1 >= t
        % Const Velo before P
        p = A + v_ab*(t - t_a);
        v = v_ab;
        % a = 0, assigned above
        break;
    end
    
    % Special Check:  t is greater than first point but less than first
    % point + tau
    if j == 2 && t < t0 && t > t_a
        p_2 = A + v_ab*tau_list(1);
        a_23 = v_ab/(2*tau_list(1));
        
        p = p_2 + v_ab*(t - t_a) + 0.5*a_23*(t - t_a)^2;
        v = v_ab + a_23*(t - t_a);
        a = a_23;
        break
    end
    
    % Constant Acceleration Check:  Check if t lies on the curved
    % region 'near' the point
    t2 = trajectory(j, 1) + tau_list(j);
    C = trajectory(j + 1, 2:end);
    t_c = trajectory(j + 1, 1);
    v_bc = (C-B)/(t_c - t_b);
    if t1 < t && t2 >= t
        % Const Accel near B:  Need C, v_bc, a_23
        a_23 = (v_bc - v_ab)/(2*tau_list(j));
        p_2 = B - v_ab*tau_list(j);
        
        p = p_2 + v_ab*(t - t1) + .5*a_23*(t-t1)^2;
        v = v_ab + a_23*(t - t1);
        a = a_23;
        break;
    end
    
    % Const Velocity Segment: Check if t is after the curve
    t3 = trajectory(j + 1, 1) - tau_list(j + 1);
    if t2 < t && t3 >= t
        % Const Velo After P
        p = B + v_bc*(t - t_b);
        v = v_bc;
        % a = 0, assigned above
        break;
    end
end

% Need Special case if greater than the Last Point + Tau and less than the
% time of the last point
t_f = trajectory(n , 1) - tau_list(n);
if t < trajectory(n, 1) && t >= t_f && j == (n - 1)
    p_2 = B + v_bc*tau_list(n);
    a_23 = v_bc/(2*tau_list(n));
    
    p = p_2 + v_bc*(t-t_f) + a_23*(t - t_f)^2;
    v = v_bc + a_23*(t - t_f);
    a = a_23;
end

end

