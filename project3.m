clear;

%% Load Data that we'll need
load points3D.mat
run('Geometry.m')

th_last = zeros(6,1);
trajectory = zeros(26, 7);
trajectory(1, :) = [0, th_last'];

H_init = dhFwdKine(linkList, th_last');
[th1, th2, th3, th4, th5, th6, r] = abbInvKine(H_init, th_last);
trajectory(2,:) = [1, th1, th2, th3, th4, th5, th6];

H_this = [-1, 0, 0, -0.01; 0, 0, 1, -.3; 0, -1, 0, 0.48; 0,0,0,0];
[th1, th2, th3, th4, th5, th6, r] = abbInvKine(H_this, trajectory(2,2:7));
trajectory(3,:) = [3, th1, th2, th3, th4, th5, th6];

xx = linspace(3, 14, 23);
H_1 = [1,0,0,0.03;0,1,0,0;0,0,1,0;0,0,0,1];
H_1 = H_this * H_1;
[th1, th2, th3, th4, th5, th6, r] = abbInvKine(H_1, trajectory(3,2:7));
trajectory(4,:) = [xx(2), th1, th2, th3, th4, th5, th6];

H_cell = cell(21,1);
H_cell{1,1} = H_1;
for i = 5:24
   th_last = trajectory(i-1, 2:7); 
    
   d_from = points3D(i-3,:);
   d_to = points3D(i-2, :);
   D = d_to - d_from;
   theta = atan2(D(1,3), D(1,1));
   ct = cos(theta);
   st = sin(theta);
   R = [ct, -st, 0; 0, 0, -1; st, ct, 0];
   H = [R, points3D(i-2,:)'; 0,0,0,1];
   H_cell{i-3,1} = H;

   [th1, th2, th3, th4, th5, th6, r] = abbInvKine(H, th_last);

   trajectory(i,:) = [xx(i-2), th1, th2, th3, th4, th5, th6];

end

trajectory(25,:) = [16, th1, th2, th3, th4, th5, th6];

H_init = dhFwdKine(linkList, [0;0;0;0;0;0]);
[th1, th2, th3, th4, th5, th6, r] = abbInvKine(H_init, [0;0;0;0;0;0]);
trajectory(26,:) = [20, th1, th2, th3, th4, th5, th6];

% p = zeros(100, 7);
% v = zeros(100, 7);
% a = zeros(100, 7);
% x_interp = linspace(0,20,100);
% 
% for i = 1:100
%     p(i,1) = x_interp(i);
%     v(i,1) = x_interp(i);
%     a(i,1) = x_interp(i);
%     [p(i, 2:7), v(i, 2:7), a(i, 2:7)] = constAccelInterp(x_interp(i), trajectory, .5);
% end

hold on
view(-30,45)
plot3(points3D(:,1),points3D(:,2),points3D(:,3));
%plotArm(p,false,gca);
plotArm([0 0 0 0 0 0],false,gca);
xlabel('x')
ylabel('y')
zlabel('z')
hold off