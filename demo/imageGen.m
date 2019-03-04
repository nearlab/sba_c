function [I] = imageGen(X,pos,R,K)
%imgageGen creates a pseudo image of points X
%   INPUTS:
%   
%   X - 3 x N array of feature locations in world frame. N features
%   pos 3x1 vector of location of camera center in world frame
%   R 3x3 rotation matrix mapping from inertial to camera frame
%   K 3x3 camera calibration matrix
%   
%   OUTPUTS:
%   
%   I - 2xN image info containing pixel coordinates. NaN if a given feature
%   is not within the frame defined by 2X the principal points

N = size(X,2);

%initialize z for homogenous coordinates
z = zeros(3,N);

%initialize I (inhomogenous coordinates)
I = zeros(2,N);

%create t (vector from camera to world center in camera frame)
t = -R*pos;

for ii = 1:N
    z(:,ii) = K*[R t]*[X(:,ii)' 1]';
    I(:,ii) = z(1:2, ii)/z(3, ii);

    %elimate points which are outside of the camera frame
    if (I(1,ii) > 2*K(1,3)) || (I(2,ii) > 2*K(2,3)) || ...
            (I(1,ii) < 0) || (I(2,ii) < 0)
        I(1:2,ii) = [NaN NaN]';
    end
end
end

