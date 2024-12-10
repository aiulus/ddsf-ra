function [u, y] = deepc2Opt(lookup, H, u_ini, y_ini)
    % Extract DeePC parameters
    Q = lookup.deepc.Q;
    R = lookup.deepc.R;
    lambda_g = lookup.deepc.lambda_g;
    
    % Extract dimensions
    m = lookup.dims.m;
    p = lookup.dims.p;
    
    % Extract configuration parameters
    T = lookup.config.T;
    T_ini = lookup.config.T_ini;
    T_f = lookup.config.T_f;

    % Extract (past/future slices of) Hankel matrices
    Up = H.Up; Yp = H.Yp;
    Uf = H.Uf; Yf = H.Yf;

    % Define symbolic variables
    g = sdpvar(T - T_ini - T_f + 1, 1);
    u = sdpvar(T_f * m, 1);
    y = sdpvar(T_f * p, 1);

    % Define the optimization objective
    objective = g' * (lambda_g *  eye(length(g))) * g ...
                + y' * kron(eye(T_f), Q) * y + ...
                u' * kron(eye(T_f), R) * u;

    % Define the constraints
    constraints = [Up * g == u_ini, ...
                   Yp * g == y_ini, ...
                   Uf * g == u, ...
                   Yf * g == y];
                       
    options = sdpsettings('solver', 'OSQP', ...          % Use OSQP solver
                      'verbose', 0, ...             % Suppress solver output
                      'osqp.max_iter', 30000, ...   % Set maximum iterations
                      'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                      'warmstart', 0);             % Disable warm start

    % options = sdpsettings('solver', 'fmincon', 'verbose', 0);
    diagnostics = optimize(constraints, objective, options);

    if diagnostics.problem == 0
                % g = value(g);
                u = value(u);
                y = value(y);
    else
        error('The problem did not solve successfully.');
    end
end