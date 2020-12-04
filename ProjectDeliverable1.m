%% Scale Factor
load points2D.mat

% Need to take local coordinates in (x, z) plane and scale them to the
% world-frame whiteboard coordinate system.  
% Writing Direction is World Frame X.  Local Coordinates Z are along the
% World Y, and Local X pointed in the direction of the next point

% The difference in X Local is 7.5
% The difference in X World is .15

% The difference in Z Local is 4
% The difference in Z World coordinates is .08

% That means that the Scaling factor of Local -> World is:
% Moving .02 in the world is the same as moving 1 in local
scaling_factor = 1/50;

%% Transform Scaled Letter to World

% To translate the points into the world frame, we need to scale the world
% coordinates by 1/50 then translate the points by x = -0.06, y = -0.3, and
% z = 0.40.  We can do this in one transform if we use homogenous points,
% using a 3x3 scaling factor matrix, a 3x1 displacement vector. and the
% homogenous vector on the bottom row
xyz_trans = [-0.06; -0.3; 0.4];

s_m = [scaling_factor, 0, 0, xyz_trans(1,1); 0, scaling_factor, 0, xyz_trans(2,1); 0, 0, scaling_factor, xyz_trans(3,1)];
scaled_points = zeros(22, 3);
for i = 1:22
    p = [points_all(i, 1); 0; points_all(i, 2); 1];
    scaled_points(i, :) = (s_m * p);
end
points3D = scaled_points;

%% Plot the Final Points

% For the orientation to be correct, we need to have the x axis directed at
% the next point and the Z oriented along the world Y.
H = cell(22, 1);
D = zeros(22,3);
u = zeros(3, 22);
v = zeros(3, 22);
w = zeros(3, 22);

% Generate the Homogenous transforms for all the points
for i = 1:21
    d_from = scaled_points(i, :);
    d_to = scaled_points(i+1, :);
    D(i, :) = d_to - d_from;
    
    theta = atan2(D(i,3), D(i,1));
    ct = cos(theta);
    st = sin(theta);
    R = [ct, -st, 0; 0, 0, -1; st, ct, 0];
    H{i, 1} = [R, D(i, :)'; 0, 0, 0, 1];
    u(:, i) = R(1, :)';
    v(:, i) = R(2, :)';
    w(:, i) = R(3, :)';
end
H{22, 1} = H{21, 1};  % Last point is same as the previous
u(:, 22) = R(1, :)';
v(:, 22) = R(2, :)';
w(:, 22)  = R(3, :)';

% Plot the orientations at each point
figure(1)
title('Axis of CSM Points')
xlabel('X')
ylabel('Y')
zlabel('Z')
view(-30, 45)
quiver3(scaled_points(:,1)',scaled_points(:,2)',scaled_points(:,3)', u(1,:), v(1,:), w(1,:), .1, 'r')
hold on
quiver3(scaled_points(:,1)',scaled_points(:,2)',scaled_points(:,3)', u(2,:), v(2,:), w(2,:), .1, 'g')
hold off
hold on
quiver3(scaled_points(:,1)',scaled_points(:,2)',scaled_points(:,3)', u(3,:), v(3,:), w(3,:), .1, 'b')
hold off
axis equal