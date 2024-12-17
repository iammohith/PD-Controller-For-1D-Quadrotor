function [ s_des ] = fixed_set_point(t, z_des)
% fixed_set_point
% Outputs a constant desired state [z_des; 0] except at t = 0 where it returns [0; 0]
% This function provides a desired state trajectory for a system. 
% For all times t > 0, the desired state is [z_des; 0], where:
%       z_des - Constant desired position in the z-axis.
% At time t = 0, the desired state is initialized to [0; 0].
%
% INPUT:
%   t - Current time (scalar)
%   z_des - Desired position in the z-axis (scalar)
%
% OUTPUT:
%   s_des - Desired state vector [position; velocity], where:
%       s_des(1) -> Desired position (z-axis)
%       s_des(2) -> Desired velocity (set to 0)

% Check if time is zero
if t == 0
    s_des = [0; 0];  % Initial state at time t=0 is [0; 0]
else
    s_des = [z_des; 0];  % Desired position is z_des and velocity is 0
end

end