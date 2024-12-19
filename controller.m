function [u] = controller(~, s, s_des, params)
% controller  
% Proportional-Derivative (PD) controller for height control
%
% Inputs:
%   s      - 2x1 vector containing the current state [z; v_z]
%   s_des  - 2x1 vector containing the desired state [z_des; v_z_des]
%   params - Structure containing robot parameters:
%       params.gravity     - Gravitational acceleration [m/s^2]
%       params.mass        - Mass of the robot [kg]
%       params.u_min       - Minimum thrust limit [N]
%       params.u_max       - Maximum thrust limit [N]
%
% Output:
%   u - Control input (thrust) to regulate height [N]

% Extract robot parameters
g = params.gravity;      % Gravitational acceleration
m = params.mass;         % Mass of the robot
u_min = params.u_min;    % Minimum thrust limit
u_max = params.u_max;    % Maximum thrust limit

% Controller gains (tuned for desired performance)
kp = 120;                % Proportional gain
kv = 20;                 % Derivative gain

% Desired acceleration (z_ddot): Set to zero (default assumption)
z_ddot = 0;

% Compute error terms
e = s_des - s;           % Error: [position_error; velocity_error]

% PD control law for thrust
u = m * (z_ddot + kp * e(1) + kv * e(2) + g);

% Enforce actuator limits (saturation)
u = max(min(u, u_max), u_min);

end
