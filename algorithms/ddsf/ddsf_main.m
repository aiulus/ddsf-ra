%% Setup
%sys = linear_system("cruise_control"); ... % Specify system
sys = ddsf_systems("dampler", true); % Set 2nd arg. to true to discretize

IO_params = struct( ...
    'debug_toggle', true, ...
    'save_to_file', true, ...
    'log_interval', 1 ... % Log every n-the step of the simulation
    );

dims = sys.dims;
T_sim = 50;
lookup = struct( ...
                'sys', sys, ...
                'sys_params', sys.params, ...
                'config', sys.config, ...
                'dims', dims, ...
                'IO_params', IO_params, ...
                'T_sim', T_sim ...
                );

%% Generate data & Hankel matrices
[u_d, y_d, x_d, ~, ~] = ddsfGenerateData(lookup); 
[H_u, H_y] = ddsf_hankel(u_d, y_d, sys);
lookup.dims.hankel_cols = size(H_u, 2);
lookup.H = [H_u; H_y];

logs = struct( ...
        'u', [u_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.m, T_sim).'].', ... % Initialize input history with the last T_ini steps of the offline input data
        'y', [y_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.p, T_sim).'].', ... % Initialize output history with the last T_ini entries of the offline data
        'x', [x_d(:, end - lookup.config.T_ini + 1: end).'; ...
        zeros(dims.n, T_sim).'].' ... % TODO: tracking x not necessary
    );

for t=(lookup.config.T_ini+1):(lookup.config.T_ini + 1 + T_sim)
    fprintf("----------------- DEBUG Information -----------------\n");
    fprintf("CURRENT SIMULATION STEP: t = %d\n", t - lookup.config.T_ini);
    u_ini = logs.u(:, (t - lookup.config.T_ini):(t-1));
    y_ini = logs.y(:, (t -lookup.config.T_ini):(t-1));
    traj_ini = [u_ini; y_ini];
    u_l = learning_policy();

    [u_opt, y_opt] = ddsf_opt(lookup, u_l, traj_ini);
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