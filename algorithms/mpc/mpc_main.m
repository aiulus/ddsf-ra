%% EXPERIMENTAL PIPELINE FOR LTI SYSTEMS USING MPC
% Misc. parameters
save_to_file = false;

% Specify the LTI system type
lti_system_description = 'double_integrator'; % Options: 'single_integrator', 'double_integrator', 'mass_spring_dampler', 'dc_motor'
% sys = LTI(lti_system_description); % Load the LTI system

system_description = 'cruise_control';
sys = linear_system(system_description);

% Extract system matrices
A = sys.A;
B = sys.B;
C = sys.C;
D = sys.D;

% MPC Parameters
dt = sys.params.dt; % Sampling time from system parameters
t_p = sys.deepc_config.N; % Prediction horizon (steps)
t_c = 5; % Control horizon (steps)

% Create the plant as a state-space object
plant = ss(A, B, C, D);

% Initialize the MPC controller
mpc_controller = mpc(plant, dt, t_p, t_c);

% Set constraints
mpc_controller.MV.Min = sys.constraints.U(1); % Minimum input
mpc_controller.MV.Max = sys.constraints.U(2); % Maximum input

if isfield(sys.constraints, 'Y') && ~isempty(sys.constraints.Y)
    mpc_controller.OV.Min = sys.constraints.Y(1); % Minimum output (if defined)
    mpc_controller.OV.Max = sys.constraints.Y(2); % Maximum output (if defined)
end

% Tuning parameters
mpc_controller.Weights.ManipulatedVariables = 1e-4; % Penalize input magnitude
mpc_controller.Weights.ManipulatedVariablesRate = 1e-3; % Penalize rate of input change
mpc_controller.Weights.OutputVariables = 1; % Penalize output deviation

% Simulation parameters
t_final = 20; % Total simulation time [s]
N_simulation = t_final / dt; % Number of time steps
x = sys.params.x_ini; % Initial state from system definition
x_history = zeros(size(A, 1), N_simulation); % Storage for states
u_history = zeros(size(B, 2), N_simulation); % Storage for control inputs

% Target reference
r = sys.target;

% Simulation loop
for k = 1:N_simulation
    % Compute optimal control action
    u = mpcmove(mpc_controller, mpcstate(mpc_controller), x, r);
    
    % Update system state
    x = A * x + B * u;

    % Log state and input
    x_history(:, k) = x;
    u_history(:, k) = u;
end

% Time vector
t = (0:N_simulation-1) * dt;

% Plot system states over time
figure; hold on;
plot(t, x_history', 'LineWidth', 1.5);
if size(x_history, 1) == 1
    yline(r, '--r', 'Target');
else
    plot(t, repmat(r(:), 1, N_simulation)', '--r', 'LineWidth', 1.5);
end
%xlabel('Time (s)');
%ylabel('States');
%legend(sys.params.state_name, 'Target');
%title(sprintf('System States for "%s"', system_description));
%hold off;

% Plot control inputs over time
%figure; hold on;
%plot(t, u_history', 'LineWidth', 1.5);
%xlabel('Time (s)');
%ylabel('Control Input');
%legend(sys.params.input_name);
%title(sprintf('Control Inputs for "%s"', system_description));
%hold off;

% Time vector
t = (0:N_simulation-1) * dt;

% Create figure for states and inputs
figure;

% Plot system states over time
subplot(2,1,1); hold on;
plot(t, x_history', 'LineWidth', 1.5);
if size(x_history, 1) == 1
    yline(r, '--r', 'Target', 'LineWidth', 1.5);
else
    plot(t, repmat(r(:), 1, N_simulation)', '--r', 'LineWidth', 1.5);
end
xlabel('Time (s)');
ylabel('States');
legend(sys.params.state_name, 'Target', 'Location', 'Best');
title(sprintf('System States for "%s"', system_description));
grid on;
hold off;

% Plot control inputs over time
subplot(2,1,2); hold on;
plot(t, u_history', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Control Input');
legend(sys.params.input_name, 'Location', 'Best');
title(sprintf('Control Inputs for "%s"', system_description));
grid on;
hold off;

% Save the figure if needed
if save_to_file
    saveas(gcf, sprintf('%s_MPC_Plots.png', system_description));
end
