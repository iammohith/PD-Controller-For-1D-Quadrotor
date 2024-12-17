function quad_state = simStateToQuadState(sim_state)
% simStateToQuadState 
% Convert simulation state to the 13-element quad state
% This function converts a simulation state (e.g., from a simulation output) 
% into the 13-element state vector used for the quadrotor system. 
% The resulting state vector follows the structure:
% [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
%
% INPUT:
%   sim_state - A 2x1 vector containing the [z, zd] values from the simulation:
%       sim_state(1) -> z position (altitude)
%       sim_state(2) -> zd velocity (rate of change of altitude)
%
% OUTPUT:
%   quad_state - A 13x1 vector representing the quadrotor state, with the following elements:
%       quad_state(1) -> x position (fixed at 0 in this implementation)
%       quad_state(2) -> y position (fixed at 0 in this implementation)
%       quad_state(3) -> z position (altitude from simulation state)
%       quad_state(4) -> x velocity (fixed at 0 in this implementation)
%       quad_state(5) -> y velocity (fixed at 0 in this implementation)
%       quad_state(6) -> z velocity (from simulation state)
%       quad_state(7) -> qw (scalar part of the quaternion, fixed at 1)
%       quad_state(8) -> qx (x component of quaternion, fixed at 0)
%       quad_state(9) -> qy (y component of quaternion, fixed at 0)
%       quad_state(10) -> qz (z component of quaternion, fixed at 0)
%       quad_state(11) -> p (pitch rate, fixed at 0)
%       quad_state(12) -> q (roll rate, fixed at 0)
%       quad_state(13) -> r (yaw rate, fixed at 0)

% Initialize the quadrotor state vector with zeros
quad_state = zeros(13,1);

% Assign values to the state vector
quad_state(1) = 0;    % x position (fixed at 0)
quad_state(2) = 0;    % y position (fixed at 0)
quad_state(3) = sim_state(1);  % z position (altitude from sim_state)
quad_state(4) = 0;    % x velocity (fixed at 0)
quad_state(5) = 0;    % y velocity (fixed at 0)
quad_state(6) = sim_state(2);  % z velocity (from sim_state)
quad_state(7) = 1;    % qw (fixed as 1 as it scalar part of the quaternion)
quad_state(8) = 0;    % qx (fixed as 0, representing no rotation)
quad_state(9) = 0;    % qy (fixed as 0, representing no rotation)
quad_state(10) = 0;   % qz (fixed as 0, representing no rotation)
quad_state(11) = 0;   % p (fixed at 0, no pitch rate)
quad_state(12) = 0;   % q (fixed at 0, no roll rate)
quad_state(13) = 0;   % r (fixed at 0, no yaw rate)

end