% Demo Point Visualization
% 
% I want to see if I can recreate the images shown in the 7 pts demo

clear
close all
clc

%load image information
sPts = dlmread('7pts.txt',' ');

%extract estimated point information
est_points = sPts(:,1:3);

%read number of frames each point is visible in
numFrames = sPts(:,5);

%extract image info
imgRaw = sPts(:,7:end);

%number of points
N = size(sPts,1);

%load frames estimated parameters
sCams = dlmread('7cams.txt',' ');

%number of frames
M = size(sCams,1);

%initialize frames
frms = -1*ones(2,N,M);

framIdxs = cell(N,1);

%parse image data
for ii = 1:N
    for jj = 1:3:numFrames(ii)*3
        
        %which frame is this point visbile in (add one to account for zero
        %indexing
        cFrm = imgRaw(ii,jj)+1;
        
        %save for later
        framIdxs{ii} = [framIdxs{ii} cFrm];
        
        %write information
        frms(1:2,ii,cFrm) = [imgRaw(ii,jj+1) imgRaw(ii,jj+2)]';
    end
    
end

%load camera calibration matrix
K = dlmread('calib.txt',' ');

%manually purge bad data
K = K(:,[1 8 9]);
K(3,3) = 1;

%principal points
p1 = K(1,3);
p2 = K(2,3);

%read sba outputed motion data
sba_mot = csvread('7pts_sba_out_mot.csv');

%read sba outputted structure data
sba_struct = csvread('7pts_sba_out.csv')';

%create camera trajectory
% camera is 6DOF
T = sba_mot(:,5:7)';

%calculate rotation matricies
R = quat2dcm(quatconj(sba_mot(:,1:4)));

%Calculate C tilde
for ii = 1:M
    C_tilde(:,ii) = -R(:,:,ii)'*[T(1,ii) T(2,ii) T(3,ii)]';
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
        z(:,jj,ii) = K*[R(:,:,ii) t(:,ii)]*[sba_struct(1,jj) sba_struct(2,jj) sba_struct(3,jj) 1]';
        z_hat(:,jj,ii) = z([1 2],jj,ii)/z(3,jj,ii);
        
        %elimate points which are outside of the camera frame in 7pts.txt
        if ~ismember(ii,framIdxs{jj})
            z_hat(:,jj,ii) = NaN*z_hat(:,jj,ii);
        end
        
        
    end
end

%create initial images with 

%plot images
figure
for ii = 1:M
    subplot(4,2,ii)
    scatter(frms(1,:,ii),frms(2,:,ii),5,'filled')
    hold on
    scatter(z_hat(1,:,ii),z_hat(2,:,ii),5,'filled')
    title(ii)
    %axis([0 2*p1 0 2*p2])
    set(gca, 'ydir','reverse')
end

legend('Input Data','Reprojected')

%plot points and estimated points
figure
scatter3(sba_struct(1,:), sba_struct(2,:), sba_struct(3,:),5,'filled')
hold on
scatter3(est_points(:,1), est_points(:,2), est_points(:,3),5,'filled')

%plot camera trajectory and estimated trajectory
plot3(C_tilde(1,:),C_tilde(2,:),C_tilde(3,:))
plot3(t(1,:),t(2,:),t(3,:))
plot3(sCams(:,5), sCams(:,6), sCams(:,7))
xlabel('x')
ylabel('y')
zlabel('z')
legend('SBA','Estimated','SBA','t','estimated')
axis equal



