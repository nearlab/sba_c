clear
%close all
%clc

%hard coded analysis of 7cams demo
%corey marcus

%Loads 3D structure data from csv file

%Load SBA generated data
sba_struct = csvread('7pts_sba_out.csv');

figure
cameratoolbar
scatter3(sba_struct(:,1),sba_struct(:,2),sba_struct(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
title('7cams Demo Results')

hold on

%Load initial estimate of structure
init_struct = csvread('7pts_struct_est.csv');

scatter3(init_struct(:,1),init_struct(:,2),init_struct(:,3))
legend('SBA','True/Estimated')

%load inital camera pose estimates
camEst = dlmread('7cams.txt',' ');

%plot initial guess at camera trajectory
plot3(camEst(:,5),camEst(:,6),camEst(:,7))

%calculate camera rotations
camRotEst = quatrotate(quatconj(camEst(:,1:4)),[0 0 1]);

%plot camera direction vecotrs
quiver3(camEst(:,5),camEst(:,6),camEst(:,7),camRotEst(:,1),camRotEst(:,2),camRotEst(:,3))

%load sba generated motion data
sba_out_mot = csvread('7pts_sba_out_mot.csv');

%plot sba generated motion data
plot3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7));

%calculate camera rotations
sba_camRot = quatrotate(quatconj(sba_out_mot(:,1:4)),[0 0 1]);

%plot camera direction vectors
quiver3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7), ...
    sba_camRot(:,1),sba_camRot(:,2),sba_camRot(:,3));

axis equal


