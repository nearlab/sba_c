function [refined_cams, refined_structure] = sba(cams, structure, images, calibration)
%sba Sparse bundle adjustment
%   Implements the sparse bundle adjustment package. This is a retrofitting
%   of the demo program contained in that package. It is acceptable for
%   offline implementation but should not be used for flight code!
%
%    Use: Using this requires successful compilation of Corey's version of
%    the sba package.
%
%    Inputs:
%
%     cams: an Mx7 arrary containing information about camera location
%     estimates with respect to the inertial frame. M is number of cameras
%     to be bundle adjusted. Organized with quaternion (scalar first)
%     followed by position estimate
%
%     structure: an Nx3 array containing estimates of the location of each
%     feature in the inertial frame. N is the number of features.
%
%     images: a 2xNxM array containing pixel locations of each feature in
%     each frame. If a given feature is not visible in a given frame it's
%     coordinates should be specified as NaN.
%
%     calibration: the 3x3 camera calibration matrix
%
%     Outputs:
% 
%     refined_cams: refined camera motion estimates, same format as cams
% 
%     refined_structure: refined feature location estimates, same format as
%     structure.
% 
%     Author: Corey Marcus cmarcus@utexas.edu


%grab M and N
M = size(cams,1);
N = size(structure,1);

%sba package wants camera quaternions to be rotation from cam to intertial
%frame. Similarly, camera positions are from cam to intertial origin in cam
%coordinate frames

R = quat2dcm(cams(:,1:4));

C_tilde = cams(:,5:7);
X_SBA = zeros(M,3);

for ii = 1:M
    X_SBA(ii,:) = (-R(:,:,ii)*C_tilde(ii,:)')';
end

cams(:,5:7) = X_SBA;
cams(:,1:4) = quatconj(cams(:,1:4));

%write calibration information file
K = calibration;
f_calib = fopen('calib.txt','w');
for ii = 1:3
    fprintf(f_calib,'%3.1f %3.1f %3.1f\n',K(ii,1),K(ii,2),K(ii,3));
end
fclose(f_calib);

%write structure estimates and image info
f_data = fopen('data.txt','w');

%initialize data
data = strings(N,4+M*3);

%write estimated point locations
for ii=1:N
    for jj = 1:3
        data(ii,jj) = num2str(structure(ii,jj),'%f');
    end
end

%count number of frames in which each point occurs
frame_count = zeros(1,N);

%we'll also figure out which frames those are
frame_idx = zeros(M,N);

z_hat = images;

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
Cam_Est = cams; %asinine, i know
[Rows, Cols] = size(Cam_Est);

cam_est_c = fopen('cam_data.txt','w');

for ii = 1:Rows
    fprintf(cam_est_c,'%f',Cam_Est(ii,1));
    for jj = 2:Cols
        fprintf(cam_est_c,' %f',Cam_Est(ii,jj));
    end
    fprintf(cam_est_c,'\n');
end

% %call bash command to execute sba file
% ! ./eucsbademo cam_data.txt data.txt calib.txt
% 
% %load SBA generated data
% refined_structure = csvread('Structure_Data.csv');
% refined_cams = csvread('Motion_Data.csv');
% 
% %trim some extraneous info
% refined_cams = refined_cams(:,1:7);

refined_cams = cams;
refined_structure = structure;


%clean up the .txt files
delete Structure_Data.csv Motion_Data.csv cam_data.txt data.txt calib.txt;

end

