classdef QuadPlot < handle
    % QuadPlot 
    % Visualization class for quadrotor dynamics and position
    % This class handles the visualization of a quadrotor, including its 
    % position, rotation, motor positions, and history over time.

    properties (SetAccess = public)
        k = 0;              % Iteration counter (time step index)
        qn;                 % Quadrotor number
        time = 0;           % Current simulation time
        state;              % Current state vector: [x, y, z, vx, vy, vz, qw, qx, qy, qz]
        rot;                % Rotation matrix (body frame to world frame)
        
        color;              % Color of the quadrotor in the plot
        wingspan;           % Distance between motors (wingspan length)
        height;             % Height of the quadrotor body
        motor;              % Motor positions in the world frame

        state_hist;         % State history (positions over time)
        time_hist;          % Time history
        max_iter;           % Maximum number of iterations to store in history
    end

    properties (SetAccess = private)
        h_3d;               % Handle for the 3D axis
        h_m13;              % Handle for motors 1 and 3
        h_m24;              % Handle for motors 2 and 4
        h_qz;               % Handle for the z-axis line of the quadrotor
        h_qn;               % Handle for displaying the quadrotor number
        h_pos_hist;         % Handle for the position history plot
        text_dist;          % Offset distance for displaying the quadrotor number
    end

    methods
        % Constructor: Initialize quadrotor visualization
        function Q = QuadPlot(qn, state, wingspan, height, color, max_iter, h_3d)
            % INPUTS:
            %   qn       - Quadrotor number
            %   state    - Initial state vector [x; y; z; vx; vy; vz; qw; qx; qy; qz]
            %   wingspan - Length of the quadrotor arms (distance between motors)
            %   height   - Height of the quadrotor body
            %   color    - Color for the visualization
            %   max_iter - Maximum number of states to store in history
            %   h_3d     - Optional: Handle to 3D axis for plotting
            
            Q.qn = qn;
            Q.state = state;
            Q.wingspan = wingspan;
            Q.color = color;
            Q.height = height;
            Q.rot = QuatToRot(Q.state(7:10)); % Convert quaternion to rotation matrix
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
            Q.text_dist = Q.wingspan / 3; % Offset distance for displaying quadrotor number

            Q.max_iter = max_iter;
            Q.state_hist = zeros(6, max_iter); % Preallocate state history matrix
            Q.time_hist = zeros(1, max_iter);  % Preallocate time history vector

            % Initialize 3D plot
            if nargin < 7, h_3d = gca; end % Use current axes if not provided
            Q.h_3d = h_3d;

            % Plot quadrotor components
            hold(Q.h_3d, 'on');
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), 'r.');
            Q.h_m13 = plot3(Q.h_3d, ...
                Q.motor(1, [1, 3]), Q.motor(2, [1, 3]), Q.motor(3, [1, 3]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            Q.h_m24 = plot3(Q.h_3d, ...
                Q.motor(1, [2, 4]), Q.motor(2, [2, 4]), Q.motor(3, [2, 4]), ...
                '-ko', 'MarkerFaceColor', Q.color, 'MarkerSize', 5);
            Q.h_qz = plot3(Q.h_3d, ...
                Q.motor(1, [5, 6]), Q.motor(2, [5, 6]), Q.motor(3, [5, 6]), ...
                'Color', Q.color, 'LineWidth', 2);
            Q.h_qn = text(...
                Q.motor(1, 5) + Q.text_dist, ...
                Q.motor(2, 5) + Q.text_dist, ...
                Q.motor(3, 5) + Q.text_dist, num2str(qn));
            hold(Q.h_3d, 'off');
        end

        % Update quadrotor state and time
        function UpdateQuadState(Q, state, time)
            % Update the quadrotor's state and time
            Q.state = state;
            Q.time = time;
            Q.rot = QuatToRot(state(7:10))'; % Update rotation matrix (body to world)
        end

        % Record the quadrotor state history
        function UpdateQuadHist(Q)
            % Increment time step index and record the current state and time
            Q.k = Q.k + 1;
            Q.time_hist(Q.k) = Q.time;
            Q.state_hist(:, Q.k) = Q.state(1:6);
        end

        % Update motor positions in the world frame
        function UpdateMotorPos(Q)
            % Recompute motor positions based on the updated state and rotation
            Q.motor = quad_pos(Q.state(1:3), Q.rot, Q.wingspan, Q.height);
        end

        % Truncate history to remove unused preallocated data
        function TruncateHist(Q)
            Q.time_hist = Q.time_hist(1:Q.k);
            Q.state_hist = Q.state_hist(:, 1:Q.k);
        end

        % Update the quadrotor visualization
        function UpdateQuadPlot(Q, state, time)
            % Update state, history, motor positions, and plot data
            Q.UpdateQuadState(state, time);
            Q.UpdateQuadHist();
            Q.UpdateMotorPos();

            % Update motor and position data in the plot
            set(Q.h_m13, 'XData', Q.motor(1, [1, 3]), 'YData', Q.motor(2, [1, 3]), 'ZData', Q.motor(3, [1, 3]));
            set(Q.h_m24, 'XData', Q.motor(1, [2, 4]), 'YData', Q.motor(2, [2, 4]), 'ZData', Q.motor(3, [2, 4]));
            set(Q.h_qz, 'XData', Q.motor(1, [5, 6]), 'YData', Q.motor(2, [5, 6]), 'ZData', Q.motor(3, [5, 6]));
            set(Q.h_qn, 'Position', [Q.motor(1, 5) + Q.text_dist, Q.motor(2, 5) + Q.text_dist, Q.motor(3, 5) + Q.text_dist]);
            set(Q.h_pos_hist, 'XData', Q.state_hist(1, 1:Q.k), 'YData', Q.state_hist(2, 1:Q.k), 'ZData', Q.state_hist(3, 1:Q.k));

            % Redraw the updated plot
            drawnow;
        end
    end
end