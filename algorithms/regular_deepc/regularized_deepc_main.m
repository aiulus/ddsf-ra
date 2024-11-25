%% Regularized DeePC Algorithm
debug_mode = true; % Toggle debug mode
save_to_file = true; % Log output if required
log_interval = 1; % Log every <log_interval> iterations

%% Parameters
T = 31; % Window length
T_ini = 5; % Initial trajectory length
N = 10; % Prediction horizon
s = 4; % Sliding length
Q = 120000; % Output cost weight
R = 0.1; % Input cost weight
lambda_g = 1; % Regularization parameter for g
lambda_y = 100; % Regularization parameter for slack variable σ_y

% Fetch system
sys = linear_system("cruise_control");

%% Step 1: Data Collection
[u_d, y_d] = generate_data(sys, T); % Simulate the system

%% Step 2: Generate Low-Rank Hankel Matrices
[Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, T_ini, N, sys);

% Reconstruct low-rank approximations
Up_approx = SVD_approx(Up);
Yp_approx = SVD_approx(Yp);
Uf_approx = SVD_approx(Uf);
Yf_approx = SVD_approx(Yf);

%% Step 3: Define Initial Conditions
u_ini = u_d(1:T_ini).';
y_ini = y_d(1:T_ini).';

if debug_mode
    disp('--- Regularized DeePC Initialization ---');
    disp('Initial Input Trajectory (u_ini):'); disp(u_ini);
    disp('Initial Output Trajectory (y_ini):'); disp(y_ini);
    disp('----------------------------------------');
end

%% Step 4: Receding Horizon Loop
max_iter = 50;
u_hist = zeros(max_iter, size(sys.B, 2)); % For storing applied inputs
y_hist = zeros(max_iter, size(sys.C, 1)); % For storing resulting outputs

ref_trajectory = sys.target * ones(N, 1); % Target trajectory

for k = 0:max_iter-1
    t = k * s + 1;
    if t > max_iter, break; end
    
    % Solve the Regularized DeePC Optimization Problem
    [g_opt, u_opt, y_opt, sigma_y] = regularized_deepc_opt(Up_approx, Yp_approx, Uf_approx, Yf_approx, ...
                                                           u_ini, y_ini, ...
                                                           ref_trajectory, Q, R, ...
                                                           sys.constraints.U, sys.constraints.Y, ...
                                                           N, lambda_g, lambda_y);
    
    % Apply the first s control inputs
    u_t = value(u_opt);
    y_t = value(y_opt);
    u_hist(t:t+s) = u_t(1:s + 1);
    y_hist(t:t+s) = y_t(1:s + 1);

    % Update Initial Trajectories
    u_ini = [u_ini(s+1:end, :); u_t(1:s)];
    y_ini = [y_ini(s+1:end, :); y_t(1:s)];

    % Log and Debug
    debug_log(t, log_interval, debug_mode, save_to_file, ...
              'u_ini', u_ini, 'y_ini', y_ini, ...
              'u_t', u_t, 'y_t', y_t, ...
              'sigma_y', sigma_y);
end

%% Final Output
% Display the applied control inputs and resulting system outputs
figure;
subplot(2,1,1);
plot(u_hist);
title('Control Inputs');
xlabel('Time'); ylabel('Control Input');

subplot(2,1,2);
hold on;
plot(y_hist, 'b');
plot(1:max_iter, ref_trajectory(1)*ones(1, max_iter), 'r--', 'LineWidth', 1.5);
title('System Outputs');
xlabel('Time'); ylabel('Output');
hold off;