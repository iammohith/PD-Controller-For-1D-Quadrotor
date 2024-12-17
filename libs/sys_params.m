function [ params ] = sys_params()
% sys_params
% Define system parameters for a quadrotor
% This function defines and returns a structure containing key physical 
% properties and actuator limits for a quadrotor system.
%
% OUTPUT:
%   params - A structure containing the following fields:
%       params.gravity   - Gravitational constant (m/s^2)
%       params.mass      - Mass of the quadrotor (kg)
%       params.arm_length - Length of the quadrotor arm (m)
%       params.u_min     - Minimum control input (thrust)
%       params.u_max     - Maximum control input (thrust)

% Physical properties
params.gravity = 9.81;              % Gravitational acceleration (m/s^2)
params.mass = 0.18;                 % Mass of the quadrotor (kg)
params.arm_length = 0.086;          % Length of one arm of the quadrotor (m)

% Actuator limits
params.u_min = 0;                   % Minimum thrust (no thrust)
params.u_max = 1.2 * params.mass * params.gravity; % Maximum thrust, a factor of the weight (to allow for dynamic control)

end