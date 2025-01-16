function [u_opt, y_opt] = singleVarOptDDSF(lookup, ul_t, traj_ini)
    verbose = lookup.IO_params.verbose;
    %% Extract parameters
    opt_params = lookup.opt_params;

    % Lengths and dimensions
    T_ini = lookup.config.T_ini;
    N = lookup.config.N;
    L = N + 2 * T_ini;
    s = lookup.config.s;
    m = lookup.dims.m;
    p = lookup.dims.p;
    num_cols = lookup.dims.hankel_cols;
    
    % Matrices
    R = lookup.opt_params.R;
    R_kron = kron(eye(N), R);
    H = lookup.H;
    H_u = lookup.H_u;
    H_y = lookup.H_y;
    T = eye(L, L);
    
    % Constraints
    U = lookup.sys.constraints.U;
    Y = lookup.sys.constraints.Y;
    u_min = U(:, 1);
    u_max = U(:, 2);
    y_min = Y(:, 1);
    y_max = Y(:, 2);

    % Initial trajectory and equilibrium states
    u_ini = traj_ini(1:m, :);
    y_ini = traj_ini(m+1:end, :);

    u_eq = lookup.sys.S_f.u_eq(1);
    y_eq = lookup.sys.S_f.y_eq(1);

    % DEBUG STATEMENT - START
    u_eq = zeros(m, 1);
    y_eq = zeros(p, 1);
    % DEBUG STATEMENT - END


    if iscell(u_eq)
        u_eq = cell2mat(u_eq);
    end
    if iscell(y_eq)
        y_eq = cell2mat(y_eq);
    end

    if opt_params.regularize
        H = regHankelDDSF(H_u, H_y);
        num_cols = size(H, 2);
    end

    %% Define symbolic variables
    alpha = sdpvar(num_cols, 1);
    u_bar = (H_u * alpha).';
    y_bar = (H_y * alpha).';
    u_bar_ini = u_bar(:, 1:T_ini);
    u_s = u_bar(:, T_ini+1 : T_ini+N);
    u_bar_fin = u_bar(:, T_ini+N+1 : 2*T_ini+N);
    y_bar_ini = y_bar(:, 1:T_ini);
    y_s = y_bar(:, T_ini+1 : T_ini+N);
    y_bar_fin = y_bar(:, T_ini+N+1 : 2*T_ini+N);

    u_flat = reshape(u_bar, [], 1);
    y_flat = reshape(y_bar, [], 1);

    u_low = repmat(u_min, 1, N + 2 * T_ini).';
    u_high = repmat(u_max, 1, N + 2 * T_ini).';
    y_low = repmat(y_min, 1, N + 2 * T_ini).';
    y_high = repmat(y_max, 1, N + 2 * T_ini).';

    constraints = [u_bar_ini == u_ini, ...
               y_bar_ini == y_ini, ...
               u_bar_fin == repmat(u_eq, 1, T_ini), ...
               y_bar_fin == repmat(y_eq, 1, T_ini) ...
    ];

    %% Define the objective function and the constraints
    delta = reshape(u_s, [], 1) - reshape(ul_t, [], 1);
    objective = delta.' * R_kron * delta;

    if lookup.opt_params.target_penalty
        target = lookup.sys.params.target;
        target(isnan(target)) = 0;
        target = repmat(target, L, 1);        
        delta_y = y_bar - target;
        delta_y = reshape(delta_y, p, L);

        discount = 0.9;
        gamma = discount .^ (L:-1:1);
        gamma = repmat(gamma, p, 1);
        delta_y = delta_y .* gamma;

        tpt = trace(delta_y * T * delta_y.');
        objective = objective + tpt;
    end

    
    switch opt_params.constr_type
        case 's' % Just enforce system behavior
            % No additional constraints needed
        case 'u' % Just encode input constraints
            constraints = [constraints, ...
                           u_flat >= u_low, ...
                           u_flat <= u_high];
        case 'y' % Just encode output constraints
            constraints = [constraints, ...
                           y_flat >= y_low, ...
                           y_flat <= y_high];
        case 'f' % All constraints
            constraints = [constraints, ...
                           u_flat >= u_low, ...
                           u_flat <= u_high, ...
                           y_flat >= y_low, ...
                           y_flat <= y_high];
    end

    %% Define solver settings and run optimization
    switch opt_params.solver_type
        case 'q'
            options = sdpsettings('verbose', verbose, 'solver', 'quadprog');
        case 'f'
            options = sdpsettings('verbose', verbose, 'solver', 'fmincon');
        case 'o'
            options = sdpsettings('solver', 'OSQP', ...
                  'verbose', verbose, ...             % Detailed solver output
                  'osqp.max_iter', 30000, ...   % Set maximum iterations
                  'osqp.eps_abs', 1e-5, ...     % Absolute tolerance
                  'osqp.eps_rel', 1e-5, ...     % Relative tolerance
                  'warmstart', 0);             % Disable warm start
        case 'b'
            options = sdpsettings('solver', 'bmibnb', 'verbose', verbose);
    end


    diagnostics = optimize(constraints, objective, options);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        u_opt = H_u * value(alpha);
        y_opt = H_y * value(alpha);
    else
        disp(diagnostics.problem); % Solver exit code
        disp(diagnostics.info);    % Detailed solver feedback        
    end

    if lookup.IO_params.verbose
        disp('---- Debug: Objective Function Evaluation ----');
        disp('Objective value:');
        disp(value(objective)); % Ensure it computes as expected
        disp('---- Debug: Feasibility Check ----');
        disp('Max constraint violation:');
        disp(max(check(constraints))); % Show largest constraint violation
    end

end
