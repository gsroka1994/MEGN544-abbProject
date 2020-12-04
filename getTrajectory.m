load Trajectory_file.mat
trajectory = zeros(66,7);
xx = linspace(1, 14, 64);
for i = 1:64 
    trajectory(i+1, 1) = xx(i);
    trajectory(i+1, 2:7) = theta_list{1,i};
end

trajectory_quat = zeros(66,8);

T = dhFwdKine(linkList, [0,0,0,0,0,0]);
[k, th] = rot2AngleAxis(T(1:3, 1:3));
dq = [T(1:3,4);cos(th/2);k*sin(th/2)];
trajectory_quat(1, :) = [0,0,0,0,1,0,0,0];
trajectory_quat(66, :) = [16, dq']; 

for i = 1:64
    T = transform_list{1,i};
    [k,th] = rot2AngleAxis(T(1:3, 1:3));
    dq = [T(1:3,4);cos(th/2);k*sin(th/2)];
    trajectory_quat(i+1, :) = [xx(i), dq'];
end

% These two are ill-defined here, so hard code them
trajectory_quat(2, 5:8) = [0.7071 0 0 0];
trajectory_quat(3, 5:8) = [0.7071 0 0 0];

for i = 1:65
    dot_p = dot(trajectory_quat(i, 5:8)', trajectory_quat(i+1, 5:8)');
    if dot_p < 0
       trajectory_quat(i+1, 5:8) = -1 * trajectory_quat(i+1, 5:8); 
    end
end


% Test Interpolation
[p,v,a] = constAccelInterp(4.0317, trajectory_quat, .5);

d = p(1:3);
q = p(4:7);

% Normalize q
if norm(q) ~= 0
  q = q/norm(q);
end

% Test Getting Transform
th = 2*atan2(norm(q(2:4)),q(1));
k = q(2:4)/norm(q(2:4));
T_test = [angleAxis2Rot(k', th) d'; 0 0 0 1];
T_tmp = [quat2Rot(q) d'; 0 0 0 1];

% Test gettting omega
omega = 2*[-q(2:4)' q(1)*eye(3)+cpMap(q(2:4))]*v(4:7)';