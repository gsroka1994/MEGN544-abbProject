function plot_points()

load('points2D.mat');

figure(1)
plot(points_all(:,1),points_all(:,2))
axis([-1 max(points_all(:,1))+1 -1 max(points_all(:,2))+1])

figure(2)
plot(points_C(:,1),points_C(:,2),'r',...
     points_S(:,1),points_S(:,2),'g',...
     points_M(:,1),points_M(:,2),'b')
axis([-1 max(points_all(:,1))+1 -1 max(points_all(:,2))+1])

save('points2D','points_C','points_S','points_M','points_all')
end