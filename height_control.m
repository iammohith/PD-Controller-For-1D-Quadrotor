function [t_out, z_out] = height_control(trajhandle, controlhandle)
% height_control 
% Simulates the height control system for a quadrotor
%
% Inputs:
%   trajhandle    - Function handle for desired trajectory generator
%   controlhandle - Function handle for the controller
%
% Outputs:
%   t_out - Simulation time vector
%   z_out - Quadrotor height trajectory over time

% Add utility functions to path
addpath('libs');

% ************************ SIMULATION PARAMETERS ************************
params = sys_params;                      % Load system parameters
real_time = true;                         % Enable real-time simulation

% ************************ INITIALIZE FIGURES **************************
disp('Initializing figures...')

% Figure configuration
h_fig = figure;
sz = [1000, 600];                         % Figure size in pixels
screensize = get(0, 'ScreenSize');        % Screen resolution
xpos = ceil((screensize(3) - sz(1)) / 2); % Center figure horizontally
ypos = ceil((screensize(4) - sz(2)) / 2); % Center figure vertically
set(h_fig, 'Position', [xpos, ypos, sz]); % Set figure position

% 3D plot configuration
h_3d = subplot(1, 2, 1);
axis equal
grid on
view(0, 0);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

% 2D height-time plot configuration
h_2d = subplot(1, 2, 2);
plot_2d = plot(h_2d, 0, 0);               % Empty plot for real-time updates
grid on
xlabel('t [s]'); ylabel('z [m]');         % Time vs Height labels

quadcolors = lines(1);                    % Color for the quadrotor plot
set(gcf, 'Renderer', 'OpenGL');           % Use OpenGL renderer

% *********************** INITIAL CONDITIONS ***************************
disp('Initializing simulation...')
max_iter = 100;                           % Maximum number of iterations
starttime = 0;                            % Simulation start time (s)
tstep = 0.01;                             % Time step (s)
cstep = 0.05;                             % Image capture time interval (s)
nstep = cstep / tstep;                    % Steps per capture
time = starttime;                         % Initialize simulation time

% Desired trajectory at start and stop
des_start = trajhandle(0);                % Initial state
des_stop = trajhandle(inf);               % Final desired state
stop_pos = des_stop(1);                   % Final desired height

% Pre-allocate state and trajectory arrays
x0 = des_start;                           % Initial state [z; zd]
xtraj = nan(max_iter * nstep, length(x0)); % State trajectory
ttraj = nan(max_iter * nstep, 1);         % Time trajectory
x = x0;                                   % Current state

% Tolerances for convergence
pos_tol = 0.01;                           % Position tolerance
vel_tol = 0.01;                           % Velocity tolerance

% ************************* RUN SIMULATION ****************************
disp('Simulation Running...')

% Initialize 3D quadrotor visualization
subplot(1, 2, 1);
quad_state = simStateToQuadState(x0);
QP = QuadPlot(1, quad_state, params.arm_length, 0.05, quadcolors(1, :), max_iter, h_3d);
QP.UpdateQuadPlot(quad_state, time);
h_title = title(h_3d, sprintf('Iteration: %d, Time: %.2f s', 0, time));

% Simulation loop
for iter = 1:max_iter
    timeint = time:tstep:time + cstep;    % Time interval for integration

    tic; % Start timer for step performance

    % Solve dynamics using ode45
    [tsave, xsave] = ode45(@(t, s) sys_eom(t, s, controlhandle, trajhandle, params), ...
                           timeint, x);
    x = xsave(end, :)';                   % Update state to last step

    % Save results for trajectories
    xtraj((iter - 1) * nstep + 1:iter * nstep, :) = xsave(1:end - 1, :);
    ttraj((iter - 1) * nstep + 1:iter * nstep) = tsave(1:end - 1);

    % Update quadrotor visualization
    quad_state = simStateToQuadState(x);
    QP.UpdateQuadPlot(quad_state, time + cstep);
    set(h_title, 'String', sprintf('Iteration: %d, Time: %.2f s', iter, time + cstep));

    % Update 2D time-height plot
    set(plot_2d, 'XData', ttraj(1:iter * nstep), 'YData', xtraj(1:iter * nstep, 1));

    time = time + cstep; % Update simulation time
    t_elapsed = toc;     % Check step performance

    % Check for simulation timeout
    if t_elapsed > cstep * 500
        disp('Error: Ode solver took too long. Controller may be unstable.');
        break;
    end

    % Pause to synchronize with real-time
    if real_time && (t_elapsed < cstep)
        pause(cstep - t_elapsed);
    end
end

% ************************ TERMINATION CHECK ***************************
% Check for convergence
if abs(stop_pos - x(1)) < pos_tol && abs(x(2)) < vel_tol
    disp('Simulation completed successfully.');
else
    disp('Warning: Simulation did not converge to desired state.');
end

% Final output: time and height trajectories
t_out = ttraj(~isnan(ttraj));             % Remove unused preallocated entries
z_out = xtraj(~isnan(ttraj), 1);          % Height data corresponding to time

disp('Simulation done.');

end