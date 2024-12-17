function [quad] = quad_pos(pos, rot, L, H)
% quad_pos 
% Calculates the quadrotor's coordinates in the world frame.
% 
% INPUTS:
%   pos       - 3x1 position vector [x; y; z] representing the quadrotor position.
%   rot       - 3x3 rotation matrix representing body-to-world rotation.
%   L         - Scalar value representing the arm length of the quadrotor.
%   H         - (Optional) Scalar value for the height offset of the quad. Default is 0.05.
%
% OUTPUT:
%   quad      - 3x6 matrix containing the coordinates of key points of the quadrotor in the world frame.

% Set default value for height H if not provided
if nargin < 4
    H = 0.05; 
end

% Homogeneous transformation matrix from body frame to world frame
wHb = [rot, pos(:); 0, 0, 0, 1]; 

% Define key points of the quadrotor in body frame:
% Each column represents [x; y; z; 1] coordinates of key points
quadBodyFrame = [L,  0,  0, 1;    % Front arm
                 0,  L,  0, 1;    % Right arm
                -L,  0,  0, 1;    % Back arm
                 0, -L,  0, 1;    % Left arm
                 0,  0,  0, 1;    % Center point
                 0,  0,  H, 1]';  % Height point

% Transform the body frame points to the world frame
quadWorldFrame = wHb * quadBodyFrame;

% Extract only the x, y, z coordinates from the homogeneous coordinates
quad = quadWorldFrame(1:3, :);

end