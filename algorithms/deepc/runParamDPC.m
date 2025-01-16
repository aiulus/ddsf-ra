function logs = runParamDPC(systype, Q, R, T_ini, N, T_sim)
     %% Define the system
    sys = sysInit(systype);
    sys.opt_params.R = R;
    sys.opt_params.Q = Q;
    sys.config.T_ini = T_ini;
    sys.config.N = N;
    verbose = true; 
    save = true;
    rng(0, 'twister'); % Set seed and generator
    
    %% TODO: Ensure shape match across _systems & Opt !!
    
    %% Extract relevant parameters
    dims = sys.dims;
    config =  sys.config;
    config.T_sim = T_sim;
    config.L = config.T_ini + config.N;
    config.T = (dims.m + 1) * (config.T_ini + config.N + dims.n) - 1;

    %% !!! TODO: Either really use the multi-step version or completely remove s !!!
    s = 1; 
    config.s = s;
    %% !!!

    opt_params = sys.opt_params;
    opt_params.lambda_g = 0.01; % Should be adaptive
    
    %% Initialize containers for logging
    u_sim = zeros(dims.m, config.T_sim + 1);
    y_sim = zeros(dims.p, config.T_sim + 1);
    loss = zeros(1, config.T_sim + 1);
    
    %% Generate data
    [x_data, y_data, u_data] = deepc3generateData(sys, dims, config);
    
    data = struct( ...
         'u_data', u_data, ...
         'y_data', y_data, ...
         'x_data', x_data);
    
    %% Parameters for Hankel matrix construction
    hankel_params = struct( ...
        'u_data', u_data, ...
        'x_data', x_data, ...
        'y_data', y_data, ...
        'T_ini', config.T_ini, ...
        'N', config.N, ...
        'n', dims.n ...
        );
    
    %% Data structure that holds all relevant parameters
    lookup = struct( ...
            'dims', dims, ...
            'hankel_params', hankel_params, ...
            'config', config, ...
            'deepc', opt_params, ...
            'sys', sys, ...
            'data', data, ...
            'T_d', 0 ... % Input delay [s] / dt [s]
            );
    
    H = deepc3hankel(lookup);
    lookup.H = H;
    
    % Reshape the last Tini columns of PData, uData, and yData into column vectors
    u_ini = reshape(u_data(:, end - config.T_ini + 1:end), [], 1);
    y_ini = reshape(y_data(:, end - config.T_ini + 1:end), [], 1);
    
    %% Receding Horizon Loop
    for t=1:config.T_sim
        % Solve the quadratic optimization problem
        % [u_opt, y_opt] =  deepc3Opt(lookup, H, u_ini, y_ini);
        [u_opt, y_opt] =  singleVarOptDPC(lookup, H, u_ini, y_ini);
    
        u_ts = u_opt(1:dims.m*config.s);
        y_ts = y_opt(1:dims.p*config.s);
        loss_t = norm(y_ts - sys.params.target);
    
        % Log the resulting trajectory
        u_sim(:, t:t+s) = u_ts;
        y_sim(:, t:t+s) = y_ts;
        loss(:, t) = loss_t;
            
        % Update the initial trajectory
        u_ini = [u_ini(dims.m + 1:end, :); u_ts];
        y_ini = [y_ini(dims.p + 1:end, :); y_ts];
    
        fprintf("------------------- " + ...
                 "DeePC - Simulation step %d / %d -------------------\n", t, config.T_sim);
        if verbose
            fprintf("Distance to target: "); disp(loss_t);
        end
    end
    
    %% Plot the results
    Tsim = config.T_sim; 
    time = 0:Tsim;
    % deepc3plot(time, y_sim, u_sim, sys)

    logs = struct( ...
        'time', time, ...
        'u_sim', u_sim, ...
        'y_sim', y_sim, ...
        'loss', loss ...
        );

    %% Save the results
    if save
        save2csv(time, u_sim, y_sim);
    end
end

