% Clear workspace and close all figures
close all;
clear;

%% ******************** SIMULATION SETUP ********************
% Define desired trajectory: Fixed height setpoint
% Setting z_des = 1 (step to desired height) for fixed height setpoint
z_des = 1;

% Trajectory generator: Generates a fixed height setpoint
trajhandle = @(t) fixed_set_point(t, z_des);

% Controller handle: PD controller for height control
controlhandle = @controller;

%% ******************** RUN SIMULATION ********************
% Run the height control simulation
[t, z] = height_control(trajhandle, controlhandle);