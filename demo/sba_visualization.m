clear
close all
clc

%Loads 3D structure data from csv file

%select SBA generated data file path
file = uigetfile('*.csv');

%Load SBA generated data
S = csvread(file);

figure
scatter3(S(:,1),S(:,2),S(:,3))
xlabel('x')
ylabel('y')
zlabel('z')
title(file)

hold on

%select true/estimated data file path
file = uigetfile('*.csv');

%Load true data
T = csvread(file);

scatter3(T(:,1),T(:,2),T(:,3))
legend('SBA','True/Estimated')

%load camera estimates
disp('Select Camera Estimate File')
%file = uigetfile('*.txt');
camEst = dlmread('7cams.txt',' ');

%plot initial guess at camera trajectory
figure
plot3(camEst(:,5),camEst(:,6),camEst(:,7))
xlabel('x')
ylabel('y')
zlabel('z')

%calculate camera rotations
camRotEst = quatrotate(camEst(:,1:4),[0 0 1]);

%plot camera direction vecotrs
quiver3(camEst(:,5),camEst(:,6),camEst(:,7),camRotEst(:,1),camRotEst(:,2),camRotEst(:,3))
