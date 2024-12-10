function [u, y] = deepc2Opt(lookup, H, u_ini, y_ini)
    verbose = true; % Toggle debug mode

    %% Extract DeePC parameters
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

    % Extract system constraints
    U = lookup.sys.constraints.U;
    Y = lookup.sys.constraints.Y;
    target = lookup.sys.params.target;

    %% Define symbolic variables
    g = sdpvar(T - T_ini - T_f + 1, 1);
    u = sdpvar(T_f * m, 1);
    y = sdpvar(T_f * p, 1);

    %% Construct the reference trajectory
    target = reshape(repmat(target, T_f, 1).', [], 1);
    r = zeros(size(target));   

    nancols = isnan(target);
    numcols = ~isnan(target);
    
    % Take over decision variables where target isn't specified
    r(nancols) = y(nancols);
    r(numcols) =  target(numcols);

    %% Define the optimization objective
    objective = g' * (lambda_g *  eye(length(g))) * g ...
                + (r - y)' * kron(eye(T_f), Q) * (r - y) + ...
                u' * kron(eye(T_f), R) * u;

    % Define the constraints
    constraints = [Up * g == u_ini, ...
                   Yp * g == y_ini, ...
                   Uf * g == u, ...
                   Yf * g == y, ...
                   U(1) <= u & u <= U(2), ...
                   Y(1) <= y & y <= Y(2) ...
                   ];
                       
    options = sdpsettings('solver', 'OSQP', ...          % Use OSQP solver
                      'verbose', 1, ...             % Suppress solver output
                      'osqp.max_iter', 30000, ...   % Set maximum iterations
                      'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                      'warmstart', 0);             % Disable warm start

    % options = sdpsettings('solver', 'fmincon', 'verbose', 0);
    diagnostics = optimize(constraints, objective, options);

    if diagnostics.problem == 0
                % g = value(g);
                u = value(u);
                y = value(y);
        if verbose
            disp("REFERENCE TRAJECTORY: "); disp(r.');
            disp("DELTA: "); disp((r - y).');
            disp("PEANALTY TERM: "); disp((r - y)' * kron(eye(T_f), Q) * (r - y));
        end
    else
        error('The problem did not solve successfully.');
    end
end