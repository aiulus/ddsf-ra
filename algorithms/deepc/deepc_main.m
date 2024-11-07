%% DeePC algorithm

%% Step 0: Define global parameters
T_ini = 5; % Length of initial trajectory
N = 20; % Prediction horizon
Q = 1; % Output weight - penalizes d(y_true, y_soll)
R = 0.1; % Input weight - penalizes large control inputs

%% Step 1: Data collection
L = 50; % Length of the data sequence for training (must be at least N + T_ini!!)
% Generate [data_length] data points from an arbitrary system
sys = linear_system("simple_integrator");
[u_d, y_d] = generate_data(sys.A, sys.B, sys.C, sys.D, L);

%% Step 2: Generate the Hankel matrices
[Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, T_ini, N);

%% Step 3: Define Initial Condition
u_ini = u_d(1:T_ini).'; % Initial input trajectory
y_ini = y_d(1:T_ini).'; % Initial output trajectory

%% Step 4: Receding Horizon Loop
max_iter = 50; % Simulation steps
u_seq = zeros(max_iter, 1); % For later storage of applied inputs
y_seq = zeros(max_iter, 1); % For later storage of resulting outputs
ref_trajectory = sys.target * ones(N, 1);

for t = 1:max_iter
    % Solve the quadratic optimization problem
    % Up, Yp, Uf, Yf, u_ini, y_ini, r, Q, R, U, Y, N
    [g_opt, u_opt, y_opt] = deepc_opt(Up, Yp, Uf, Yf, ...
        u_ini, y_ini, ...
        ref_trajectory, Q, R, sys.constraints.U, sys.constraints.Y, N);

    % Apply the first optimal control input
    u_t = u_opt(1);
    u_seq(t) = u_t;

    % Simulate system response
    y_t = sys.C * y_seq(max(t - 1, 1)) + sys.D * u_t;

    % Store the output
    y_seq(t) = y_t;

    % Update the initial trajectory
    u_ini = [u_ini(:, 2:end), u_t];
    y_ini = [y_ini(:, 2:end), y_t];

    % Plot the results (optional)
    fprintf('Iteration %d: Control Input = %.2f, Output = %.2f\n', t, u_t, y_t);
end

%% Final Output
% Display the applied control inputs and resulting system outputs
figure;
subplot(2,1,1);
plot(u_sequence);
title('Applied Control Inputs');
subplot(2,1,2);
plot(y_sequence);
title('System Outputs');

