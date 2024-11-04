%% CRUISE CONTROL: CONSTANT SPEED

% Parameters
m = 1000; % Vehicle mass [kg]
b = 50; % Damping coefficient [N*s/m]
dt = 0.1; % Time step for discretization

% State-Space Matrices
A = 1 - (b * dt) / m;
B = dt / m;
C = 1;
D = 0;

% Create a state-space model for the MPC controller
plant = ss(A, B, C, D);

% MPC paraeters
t_p = 10; % Prediction horizon in steps
t_c = 2; % Control horizon in steps

% Initialize the MPC controller
mpc_controller = mpc(plant, dt, t_p, t_c);

% Initialize the MPC state
x_mpc = mpcstate(mpc_controller);

% Input constraints
mpc_controller.MV.Min = 0; % Minimum force
mpc_controller.MV.Max = 5000; % Maximim force

% Tuning parameters
mpc_controller.Weights.ManipulatedVariablesRate = 0; % Penalize changes in input
mpc_controller.Weights.OutputVariables = 1; % Penalize deviations from desired speed
mpc_controller.Weights.ManipulatedVariables = 0; % No direct penalty on input values

% DeePC parameters
T_ini = 5; % Past horizon length
N = 10; % Prediction horizon
data_length = 50; % Length of the data sequence for training
scaling_factor = data_length^2;

% Simulation parameters
t_final = 20; % Total simulation time [s]
N = t_final / dt; % Time steps
target_velocity = 20; % [m/s]
x = 15.05; % Initial state (velocity)
x_history = zeros(1, N); % For later storage of velocity values
u_history = zeros(1, N); % For later storage of control input values

% Simulation loop
for k = 1:N
    r = target_velocity; % reference signal
    u = mpcmove(mpc_controller, x_mpc, x, r); % optimal control action using MPC
    x = A*x + B*u; % Update the state
    x_history(k) = x; % Log velocity
    u_history(k) = u; % Log control input
end

% Plotting results
t = (0:N-1) * dt; % Time vector
figure; hold on;
plot(t, x_history, 'LineWidth', 1.5);
yline(target_velocity, '--r', 'Target Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Cruise Control with MPC');
legend('Vehicle Velocity', 'Target Velocity');
