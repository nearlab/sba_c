clear
%close all
%clc

%hard coded analysis of 54cams demo
%corey marcus

%Loads 3D structure data from csv file

%Load SBA generated data
S = csvread('54pts_sba_out.csv');

figure
cameratoolbar
scatter3(S(:,1),S(:,2),S(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
title('54cams Demo Results')

hold on

%Load initial estimate of structure
T = csvread('54pts_struct_est.csv');

scatter3(T(:,1),T(:,2),T(:,3))

%load inital camera pose estimates
camEst = dlmread('54cams.txt',' ');

%plot initial guess at camera trajectory
plot3(camEst(:,5),camEst(:,6),camEst(:,7))

%calculate camera rotations
camRotEst = quatrotate(quatconj(camEst(:,1:4)),[0 0 1]);

%plot camera direction vecotrs
quiver3(camEst(:,5),camEst(:,6),camEst(:,7),camRotEst(:,1),camRotEst(:,2),camRotEst(:,3))

%load sba generated motion data
sba_out_mot = csvread('54pts_sba_out_mot.csv');

%plot sba generated motion data
plot3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7));

%calculate camera rotations
sba_camRot = quatrotate(quatconj(sba_out_mot(:,1:4)),[0 0 1]);

%plot camera direction vectors
quiver3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7), ...
    sba_camRot(:,1),sba_camRot(:,2),sba_camRot(:,3));

legend('SBA Structure','Init Structure','Init Position','Init Attitude'...
    ,'SBA Position','SBA Attitude')

axis equal


