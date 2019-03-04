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
f1 = 851.57945; %camera focal length
f2 = 853.01905;
p1 = 330.24755; %principal point x
p2 = 262.19500; %principal point y

K = [f1 0 p1;
    0 f1 p2;
    0 0 1]; %camera calibration matrix

%Create jpg Images?
%imgGen = true;
imgGen = false;

%load 7pts data
load 7pts_all_data.mat;

%Number of points
N = 465;

%create array of estimated point locations
est_points = init_struct;

%initialize points
X = sba_struct';

%create camera trajectory
% camera is 6DOF
M = 7; %number of camera frames
T = sba_out_mot(:,5:7)';

%calculate rotation matricies
R = quat2dcm(quatconj(sba_out_mot(:,1:4)));

%create estimated camera matricies
Cam_Est = camEst;

%convert camera estimates to strings
Cam_Est = string(Cam_Est);

%calculate camera centers
for ii = 1:M
    C_tilde(:,ii) = -R(:,:,ii)'*sba_out_mot(ii,5:7)';
end


%create camera vectors
for ii = 1:M
    C_vect(:,ii) = R(:,:,ii)'*[0 0 1]';
end

%calculate t vector
for ii = 1:M
    t(:,ii) = -R(:,:,ii)*C_tilde(:,ii);
end

%create camera images
z = zeros(3,N,M);
z_hat = zeros(2,N,M);

for ii = 1:M
    for jj = 1:N
        z(:,jj,ii) = K*[R(:,:,ii) t(:,ii)]*[X(1,jj) X(2,jj) X(3,jj) 1]';
        z_hat(:,jj,ii) = z([1 2],jj,ii)/z(3,jj,ii);
        
        %elimate points which are outside of the camera frame
        if (z_hat(1,jj,ii) > 2*p1) || (z_hat(2,jj,ii) > 2*p2) || ...
                (z_hat(1,jj,ii) < 0) || (z_hat(2,jj,ii) < 0)
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
plot3(C_tilde(1,:),C_tilde(2,:),C_tilde(3,:),'r')
%axis([-1 3 -2 2 0 4])

%plot vectors of camera direction
quiver3(C_tilde(1,:),C_tilde(2,:),C_tilde(3,:),C_vect(1,:),C_vect(2,:),C_vect(3,:),'g')
legend('Objects','Camera Trajectory','Camera Orientation')

flipY = true;

%plot first image
figure
subplot(2,2,1)
scatter(z_hat(1,:,1),z_hat(2,:,1),5,'filled')
set(gca, 'ydir','reverse')
axis([0 1280 0 960])
title('Image 1')
axis equal

subplot(2,2,2)
scatter(z_hat(1,:,round(M/2)),z_hat(2,:,round(M/2)),5,'filled')
set(gca, 'ydir','reverse')
axis([0 1280 0 960])
title('Image 2')
axis equal

subplot(2,2,3)
scatter(z_hat(1,:,round(3*M/4)),z_hat(2,:,round(3*M/4)),5,'filled')
set(gca, 'ydir','reverse')
axis([0 1280 0 960])
title('Image 3')
axis equal

subplot(2,2,4)
scatter(z_hat(1,:,M),z_hat(2,:,M),5,'filled')
set(gca, 'ydir','reverse')
axis([0 1280 0 960])
title('Image 4')
axis equal


%write calibration information file
f_calib = fopen('calib_c.txt','w');
for ii = 1:3
    fprintf(f_calib,'%3.1f %3.1f %3.1f\n',K(ii,1),K(ii,2),K(ii,3));
end
fclose(f_calib);

%write structure estimates and image info
f_data = fopen('data_c.txt','w');

%initialize data
data = strings(N,4+M*3);

%write estimated point locations
for ii=1:N
    for jj = 1:3
        data(ii,jj) = num2str(est_points(ii,jj),'%f');
    end
end

%count number of frames in which each point occurs
frame_count = zeros(1,N);

%we'll also figure out which frames those are
frame_idx = zeros(M,N);

for ii = 1:N
    cnt = 0;
    for jj = 1:M
        if ~isnan(z_hat(1,ii,jj))
            cnt = cnt+1;
            frame_idx(cnt,ii) = jj;
        end
        frame_count(ii) = cnt;
    end
end

%create data array
for ii = 1:N
    
    %write number of frames which the point is visible in
    data(ii,4) = string(frame_count(ii));
    
    %write the location in each frame
    row = [];
    for jj = 1:frame_count(ii)
        row = [row string(frame_idx(jj,ii)-1) string(z_hat(1,ii,frame_idx(jj,ii))) string(z_hat(2,ii,frame_idx(jj,ii)))]; 
    end
    
    %insert row
    data(ii,5:4+length(row)) = row;
    
end

%write data array to file
[Rows, Cols] = size(data);

%doublespace = [3 4 7:3:Cols]

for ii = 1:Rows
    fprintf(f_data,'%s',data(ii,1));
    for jj = 2:Cols
        fprintf(f_data,' %s',data(ii,jj));
        
        if jj == 3 || jj == 4
            fprintf(f_data,' ');
        end
    end
    
    fprintf(f_data,'\n');
end
fclose(f_data);

%Write Estimated Camera Locations to File
[Rows, Cols] = size(Cam_Est);

cam_est_c = fopen('cam_data_c.txt','w');

for ii = 1:Rows
    fprintf(cam_est_c,'%f',Cam_Est(ii,1));
    for jj = 2:Cols
        fprintf(cam_est_c,' %f',Cam_Est(ii,jj));
    end
    fprintf(cam_est_c,'\n');
end

figure(2)

% %JPG image generation
% if imgGen
%     %remove existing images if they exist
%     a = rmdir('Images','s');
%     clear a;
%     
%     %Create Image Directory
%     mkdir('Images');
%     
%     %begin image population
%     for ii = 1:M
%         
%         %initialize image
%         img = zeros(2*p2,2*p1);
%         
%         for jj = 1:N
%             img(round(z_hat(2,jj,ii)),round(z_hat(1,jj,ii))) = 1;
%         end
%         
%         %create image name
%         str = strcat('Images/image',string(ii),'.jpg');
%         
%         %write image
%         imwrite(img,str);
%     end
% end