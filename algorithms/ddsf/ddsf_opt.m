function [u_opt, y_opt] = ddsf_opt(params, u_l, traj_ini)
    %% Extract parameters
    % Extract lengths and dimensions
    T_ini = params.T_ini;
    N_p = params.N_p;
    L = N_p + 2 * T_ini;
    m = params.dims.m;
    p = params.dims.p;
    num_cols = params.dims.hankel_cols;
    
    % Extract matrices
    C = params.sys.C;
    D = params.sys.D;
    T_u = params.sys.equilibrium.U;
    R = params.R;
    H = params.H;
    
    % Extract constraints
    U = params.sys.constraints.U;
    Y = params.sys.constraints.Y;
    u_min = U(:, 1);
    u_max = U(:, 2);
    y_min = Y(:, 1);
    y_max = Y(:, 2);

    
    %% Define symbolic variables
    alpha = sdpvar(num_cols, 1);
    control_u = sdpvar(m, N_p + 2 * T_ini);
    control_y = sdpvar(p, N_p + 2 * T_ini);
    % traj_p = [control_u; control_y];
    

    %% Flatten the variables 
    u_bar = reshape(control_u.', [], 1);
    y_bar = reshape(control_y.', [], 1);
    traj_p_bar = [u_bar; y_bar];

    u_bar_ini = u_bar(1:(T_ini * m));
    y_bar_ini = y_bar(1:T_ini * p);
    traj_bar_ini = [u_bar_ini; y_bar_ini];

    u_ini = traj_ini(1:m, :);
    y_ini = traj_ini(m+1:end, :);
    u_ini_flat = reshape(u_ini.', [], 1);
    y_ini_flat = reshape(y_ini.', [], 1);
    traj_ini_flat = [u_ini_flat; y_ini_flat];

    %% Define the objective function and the constraints
    delta_u = control_u(:, 1) - u_l;
    objective = delta_u.' * R * delta_u;

    constraints = [traj_p_bar == H * alpha, ...
                   traj_bar_ini == traj_ini_flat];
    % Element-wise constraints on control inputs
    constraints = [constraints, control_u >= repmat(u_min, 1, N_p + 2 * T_ini)];
    constraints = [constraints, control_u <= repmat(u_max, 1, N_p + 2 * T_ini)];
    
    % Element-wise constraints on control outputs
    constraints = [constraints, control_y >= repmat(y_min, 1, N_p + 2 * T_ini)];
    constraints = [constraints, control_y <= repmat(y_max, 1, N_p + 2 * T_ini)];
    
    % Equilibrium constraints
    YU_eq = (C * pinv(T_u) + D) * control_u;
    constraints = [constraints, YU_eq >= repmat(y_min, 1, N_p + 2 * T_ini)];
    constraints = [constraints, YU_eq <= repmat(y_max, 1, N_p + 2 * T_ini)];


    

    %% Define solver settings and run optimization
    options_quadprog = sdpsettings('verbose', 0, 'solver', 'quadprog');  
    diagnostics = optimize(constraints, objective, options_quadprog);
        
    if diagnostics.problem == 0 % Feasible solution found
        % Extract optimal values
        u_opt = value(control_u);
        y_opt = value(control_y);
    else
        % Quadprog failed, ask user for next solver choice
        disp('Quadprog failed.'); 
        disp(diagnostics.problem); % Solver exit code
        disp(diagnostics.info);    % Detailed solver feedback

        user_input = '';
        while ~ismember(lower(user_input), {'f', 'o'})
            user_input = input('Enter "f" for fmincon or "o" for osqp: ', 's');
        end
    
        if lower(user_input) == 'f' % User chose fmincon
            disp('Trying with fmincon...');
            options_fmincon = sdpsettings('verbose', 1, 'solver', 'fmincon');
            diagnostics = optimize(constraints, objective, options_fmincon);
    
            if diagnostics.problem == 0 % Feasible solution found with fmincon
                % Extract optimal values
                u_opt = value(control_u);
                y_opt = value(control_y);
            else
                error('Optimization problem is infeasible even with fmincon!');
                disp(diagnostics.problem); % Solver exit code
                disp(diagnostics.info);    % Detailed solver feedback
            end
    
        elseif lower(user_input) == 'o' % User chose osqp
            disp('Trying with osqp...');
            options_osqp = sdpsettings('solver', 'OSQP', ...
                  'verbose', 0, ...             % Suppress solver output
                  'osqp.max_iter', 30000, ...   % Set maximum iterations
                  'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                  'warmstart', 0);             % Disable warm start
    
            diagnostics = optimize(constraints, objective, options_osqp);
    
            if diagnostics.problem == 0 % Feasible solution found with osqp
                % Extract optimal values
                u_opt = value(control_u);
                y_opt = value(control_y);
            else
                error('Optimization problem is infeasible even with osqp!');
                disp(diagnostics.problem); % Solver exit code
                disp(diagnostics.info);    % Detailed solver feedback
            end
        end
        disp('---- Debug: Objective Function Evaluation ----');
        disp('Objective value:');
        disp(value(objective)); % Ensure it computes as expected

        disp('---- Debug: Feasibility Check ----');
        disp('Max constraint violation:');
        disp(max(check(constraints))); % Show largest constraint violation
    end

end

