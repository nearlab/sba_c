clear
%close all
%clc

%visualize results of eucsbademo
%corey marcus

% false for hard-coded, true for chose your own file paths
fast = false;
%fast = true;

%Loads 3D structure data from csv file

%Load SBA generated data
if fast
    disp('Select SBA structure output')
    file = uigetfile('*.csv');
    S = csvread(file);
else
    S = csvread('Structure_Data.csv');
end
    
figure
cameratoolbar
scatter3(S(:,1),S(:,2),S(:,3),5,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
title('SBA Results')

hold on

%Load initial estimate of structure and image info
if fast
    disp('Select Inital Structure Estimates')
    file = uigetfile('*.txt');
    T = dlmread(file,' ');
else
    T = dlmread('data_c.txt',' ');    
end

scatter3(T(:,1),T(:,2),T(:,3),5,'filled')

%load inital camera pose estimates
if fast
    disp('Select Initial Camera Estimates')
    file = uigetfile('*.txt');
    camEst = dlmread(file,' ');
else
    camEst = dlmread('cam_data_c.txt',' ');
end


%plot initial guess at camera trajectory
plot3(camEst(:,5),camEst(:,6),camEst(:,7))

%calculate camera rotations
camRotEst = quatrotate(quatconj(camEst(:,1:4)),[0 0 1]);

%plot camera direction vecotrs
quiver3(camEst(:,5),camEst(:,6),camEst(:,7),camRotEst(:,1),camRotEst(:,2),camRotEst(:,3))

%load sba generated motion data
if fast
    disp('Select SBA generated Motion Estimates')
    file = uigetfile('*.csv');
    sba_out_mot = csvread(file);
else
    sba_out_mot = csvread('Motion_Data.csv');
    
    %trim some extraneous info
    sba_out_mot = sba_out_mot(:,1:7);
end

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

%plot camera trajectory alone
figure

%plot initial guess at camera trajectory
plot3(camEst(:,5),camEst(:,6),camEst(:,7))
hold on

%plot camera direction vecotrs
quiver3(camEst(:,5),camEst(:,6),camEst(:,7),camRotEst(:,1),camRotEst(:,2),camRotEst(:,3))

%plot sba generated motion data
plot3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7))

%plot camera direction vectors
quiver3(sba_out_mot(:,5),sba_out_mot(:,6),sba_out_mot(:,7), ...
    sba_camRot(:,1),sba_camRot(:,2),sba_camRot(:,3));

legend('Init Position','Init Attitude','SBA Position','SBA Attitude')

axis equal

%reproject images


%load calibration matrix
K = dlmread('calib_c.txt');

%convert quaternions to DCMs
R = quat2dcm(quatconj(sba_out_mot(:,1:4)));

%number of frames
M = size(R,3);

%find camera centers
for ii = 1:M
    C_tilde(ii,:) = -R(:,:,ii)'*[sba_out_mot(ii,5) sba_out_mot(ii,6) sba_out_mot(ii,7)]';
end

%number of points
N = size(S,1);

p1 = K(1,3);
p2 = K(2,3);

%calculate t vecotor
for ii = 1:M
    t(:,ii) = (-R(:,:,ii)*C_tilde(ii,:)')';
end

%create camera images
z = zeros(3,N,M);
z_hat = zeros(2,N,M);

for ii = 1:M
    for jj = 1:N
        z(:,jj,ii) = K*[R(:,:,ii) t(:,ii)]*[S(jj,1) S(jj,2) S(jj,3) 1]';
        z_hat(:,jj,ii) = z([1 2],jj,ii)/z(3,jj,ii);
        
             
        %elimate points which are outside of the camera frame
        if (z_hat(1,jj,ii) > p1+640) || (z_hat(2,jj,ii) > p2+480) || ...
                (z_hat(1,jj,ii) < p1-640) || (z_hat(2,jj,ii) < p2-480)
            z_hat(1:2,jj,ii) = [NaN NaN]';
        end
    end
end


%plot new images
%plot first image

figure(2)
subplot(2,2,1)
hold on
scatter(z_hat(1,:,1),z_hat(2,:,1),5,'filled')
legend('Original Images','Reprojected Images')

subplot(2,2,2)
hold on
scatter(z_hat(1,:,round(M/2)),z_hat(2,:,round(M/2)),5,'filled')


subplot(2,2,3)
hold on

scatter(z_hat(1,:,round(3*M/4)),z_hat(2,:,round(3*M/4)),5,'filled')

subplot(2,2,4)
hold on
scatter(z_hat(1,:,end),z_hat(2,:,end),5,'filled')
