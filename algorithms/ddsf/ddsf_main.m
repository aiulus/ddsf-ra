%% Setup
%sys = linear_system("cruise_control"); ... % Specify system
sys = nonlinear_system("ddsf_quadrotor");

IO_params = struct( ...
    'debug_toggle', true, ...
    'save_to_file', true, ...
    'log_interval', 1 ... % Log every n-the step of the simulation
    );

dims = struct( ...
    'm', sys.params.m, ...  % Input dimension
    'p', sys.params.p, ... % Output dimension
    'n', sys.params.n ... % Dim. of the minimal state-space representation
    );
    %% 

alg_params = struct( ...
    'sys', sys, ... % Specify system
    'T', sys.ddsf_config.T, ... % Data length
    'T_ini', sys.ddsf_config.T_ini, ... % Length of the initial trajectory
    'T_sim', 50, ... % Simulation steps
    'N_p', sys.ddsf_config.N_p, ... % Prediction horizon
    's', sys.ddsf_config.s, ... % Sliding length
    'R', sys.ddsf_config.R * eye(dims.m), ... % Cost matrix
    'dims', dims, ...           % For single-point parameter-passing                   
    'IO_params', IO_params ...
    );

%% Generate data & Hankel matrices
[u_d, y_d, x_d, ~, ~] = generate_data(sys, alg_params.T); 
[H_u, H_y] = ddsf_hankel(u_d, y_d, sys);
alg_params.dims.hankel_cols = size(H_u, 2);
alg_params.H = [H_u; H_y];

logs = struct( ...
        'u', [u_d(:, end - alg_params.T_ini + 1: end).'; ...
        zeros(dims.m, alg_params.T_sim).'].', ... % Initialize input history with the last T_ini steps of the offline input data
        'y', [y_d(:, end - alg_params.T_ini + 1: end).'; ...
        zeros(dims.p, alg_params.T_sim).'].', ... % Initialize output history with the last T_ini entries of the offline data
        'x', [x_d(:, end - alg_params.T_ini + 1: end).'; ...
        zeros(dims.n, alg_params.T_sim).'].' ... % TODO: tracking x not necessary
    );

for t=(alg_params.T_ini+1):(alg_params.T_ini + 1 + alg_params.T_sim)
    fprintf("----------------- DEBUG Information -----------------\n");
    fprintf("CURRENT SIMULATION STEP: t = %d\n", t - alg_params.T_ini);
    u_ini = logs.u(:, (t - alg_params.T_ini):(t-1));
    y_ini = logs.y(:, (t - alg_params.T_ini):(t-1));
    traj_ini = [u_ini; y_ini];
    u_l = learning_policy();

    [u_opt, y_opt] = ddsf_opt(alg_params, u_l, traj_ini);
    u_next = u_opt(:, 1);
    y_next = y_opt(:, 1);

    logs.u(:, t) = u_next;
    logs.y(:, t) = y_next;
end

% TODO: Remove placeholder
function u_l = learning_policy()
    %   LATER change to:
    %       u_l = learning_policy(y, y_d)
    %
    %   INPUTS:
    %       y   - Current system output (px1)
    %       y_d - Desired system output

    %u_l = randi([-1, 1], 1, 1);
    prbs = idinput(1, 'prbs', [], [-1, 1]);

    lb = - 1.5;
    ub = 1.5;
    random_magnitude = lb + (ub - lb) * rand;
    
    % Returns a pseudo-random binary signal scaled by a random magnitude
    u_l = random_magnitude * prbs;
end