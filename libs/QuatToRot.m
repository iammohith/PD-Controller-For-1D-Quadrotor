function R = QuatToRot(q)
% QuatToRot 
% Converts a Quaternion to a Rotation Matrix
% This function converts a unit quaternion to a corresponding 
% 3x3 rotation matrix.
%
% INPUT:
%   q - A 4x1 quaternion vector [q0; q1; q2; q3] where:
%       q0 is the scalar part (real part)
%       q1, q2, q3 are the vector parts (imaginary components)
%
% OUTPUT:
%   R - A 3x3 rotation matrix corresponding to the quaternion q

% Normalize quaternion to ensure it's a unit quaternion
q = q ./ sqrt(sum(q.^2));  % Divide by the norm of q to get a unit quaternion

% Initialize a matrix qahat that represents the skew-symmetric part
qahat = zeros(3, 3);  % Preallocate the skew-symmetric matrix

% Fill in the skew-symmetric matrix (cross-product matrix)
qahat(1,2) = -q(4);    % q(1) is scalar, q(2), q(3), q(4) are vector components
qahat(1,3) = q(3);
qahat(2,3) = -q(2);
qahat(2,1) = q(4);
qahat(3,1) = -q(3);
qahat(3,2) = q(2);

% Compute the rotation matrix using the formula: R = I + 2*qahat*qahat + 2*q0*qahat
R = eye(3) + 2*qahat*qahat + 2*q(1)*qahat;  % R = I + 2 * q_hat * q_hat + 2 * q0 * q_hat

end