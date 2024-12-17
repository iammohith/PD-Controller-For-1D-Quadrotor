function [ sdot ] = sys_eom(t, s, controlhandle, trajhandle, params)
% sys_eom 
% Differential equation for the height control system
% This function represents the system's equations of motion for a
% quadrotor's height control system. It calculates the rate of change
% of the system state (sdot) based on the current state (s), the desired 
% trajectory, and the control inputs.
%
% INPUT:
%   t - The current time
%   s - The current state vector [height; velocity], where:
%       s(1) -> height (z position)
%       s(2) -> vertical velocity (zd)
%   controlhandle - Function handle to the controller that computes the desired control input
%   trajhandle - Function handle to the trajectory generator that computes the desired trajectory
%   params - A structure containing system parameters, including:
%       params.u_min - Minimum control input
%       params.u_max - Maximum control input
%       params.mass - Mass of the quadrotor
%       params.gravity - Gravitational constant (typically 9.81 m/s^2)
%
% OUTPUT:
%   sdot - The rate of change of the state vector [height rate; velocity rate]
%       sdot(1) -> rate of change of height (velocity)
%       sdot(2) -> rate of change of velocity (acceleration)

% Get the desired trajectory at the current time
s_des = trajhandle(t);

% Compute the desired control input based on the current state and desired trajectory
u_des = controlhandle(t, s, s_des, params);

% Clamp the control input to be within the allowed bounds
u_clamped = min(max(params.u_min, u_des), params.u_max);

% Define the differential equation for the system
% sdot(1) -> rate of change of height (velocity)
% sdot(2) -> rate of change of velocity (acceleration)
sdot = [s(2);  % The velocity is the rate of change of height (s(1))
        u_clamped / params.mass - params.gravity];  % The acceleration is the control input divided by mass minus gravity

end