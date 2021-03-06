% Corey Marcus
% UT Austin: ASE
% NEAR Lab
% 
% I attempt to duplicate the simulation results presented in
% "Real time monocular SLAM: Why filter" by Strasdat et al

% This script creates some images for use with the Sparse Bundle
% Adjustement Package.

clear
close all
clc

%Camera properties
%Resolution = 1280x960
f = 1000; %camera focal length
p1 = 640; %principal point x
p2 = 480; %principal point y

K = [f 0 p1;
    0 f p2;
    0 0 1]; %camera calibration matrix

%Create jpg Images?
%imgGen = true;
imgGen = false;

%Experiment setup

%Ex 1: sideways motion
Nx = 30;
Ny = 10;
N = Nx*Ny; %total number of points
obj_x = [linspace(-.5,.5,Nx)];% linspace(0,3,Nx/2)];
obj_y = [linspace(-.75,.75,Ny)];% linspace(0,.1,Ny/2)];
obj_z = 10;

%create array of estimated point locations
est_points = [];

for ii = 1:Nx
    for jj = 1:Ny
        est_points = [est_points; obj_x(ii) obj_y(jj) obj_z];
    end
end

%add a little bit of noise to the estimated points
% est_points = est_points + .01*randn(size(est_points));

%initialize points
X = zeros(3,N);
kk = 1;

for ii = 1:Nx
    for jj = 1:Ny
        X(:,kk) = [obj_x(ii) obj_y(jj) obj_z]';
        kk = kk + 1;
    end
end

%create camera trajectory
% camera is 6DOF
M = 10; %number of camera frames
T = [linspace(0,1,M);
    linspace(0,.4,M);
    linspace(0,.1,M);
    linspace(0,.2,M);
    linspace(0,.3,M);
    linspace(0,.1,M)];

%calculate rotation matricies
R = angle2dcm(T(4,:),T(5,:),T(6,:),'XYZ');

%create quaternions
Q_SBA = dcm2quat(R);

%isolate C_tilde
C_tilde = [T(1,:)' T(2,:)' T(3,:)'];

X_SBA = T(1:3,:)';

%create estimated camera matricies
Cam_Est = [Q_SBA X_SBA];

%add a small amount of noise to camera estimations
Cam_Est = Cam_Est + .01*randn(size(Cam_Est));

%create camera vectors
for ii = 1:M
    C_vect(:,ii) = R(:,:,ii)'*[0 0 1]';
end

%calculate t vector
for ii = 1:M
    t(:,ii) = -R(:,:,ii)*C_tilde(ii,:)';
end

%create camera images
z = zeros(3,N,M);
z_hat = zeros(2,N,M);

for ii = 1:M
    for jj = 1:N
        z(:,jj,ii) = K*[R(:,:,ii) t(:,ii)]*[X(1,jj) X(2,jj) X(3,jj) 1]';
        z_hat(:,jj,ii) = z([1 2],jj,ii)/z(3,jj,ii);
        
             
        %elimate points which are outside of the camera frame
        if (z_hat(1,jj,ii) > p1+640) || (z_hat(2,jj,ii) > p2+480) || ...
                (z_hat(1,jj,ii) < p1-640) || (z_hat(2,jj,ii) < p2-480)
            z_hat(1:2,jj,ii) = [NaN NaN]';
        end
    end
end

%Plot Experiment Setup
figure
cameratoolbar
scatter3(X(1,:),X(2,:),X(3,:),5,'filled')
title('Experiment Setup')
xlabel('x')
ylabel('y')
zlabel('z')

hold on
plot3(T(1,:),T(2,:),T(3,:),'r')

[cams_sba, struc_sba] = sba(Cam_Est,est_points,z_hat,K);

scatter3(struc_sba(:,1),struc_sba(:,2),struc_sba(:,3),5,'filled')


