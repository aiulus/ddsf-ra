function [u, y] = deepc2Opt(lookup, H, u_ini, y_ini)
    verbose = true; % Toggle debug mode
    optimizer_type = 'q'; % Toggle optimization type 
    constr_type = 'f'; % Toggle constraint type

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
    % TODO
    %regularized_objective = g' * (lambda_g *  eye(length(g))) * g ...
    %            + (r - y)' * kron(eye(T_f), Q) * (r - y) + ...
    %            u' * kron(eye(T_f), R) * u;
    

    %% TODO: y(~isnan(target)) - r(~isnan(target))
    objective = (y - r)' * kron(eye(T_f), Q) * (y - r) + ...
                u' * kron(eye(T_f), R) * u;
    
    u_lb = reshape(repmat(U(:, 1), T_f, 1).', [], 1);
    u_ub = reshape(repmat(U(:, 2), T_f, 1).', [], 1);
    y_lb = reshape(repmat(Y(:, 1), T_f, 1).', [], 1);
    y_ub = reshape(repmat(Y(:, 2), T_f, 1).', [], 1);

    % Define the constraints
    if lower(constr_type) == 'f'
        constraints = [Up * g == u_ini, ...
                       Yp * g == y_ini, ...
                       Uf * g == u, ...
                       Yf * g == y, ...
                       u_lb <= u & u <= u_ub, ...
                       y_lb <= y & y <= y_ub ...
                      ];
    elseif lower(constr_type) == 's'
        constraints = [Up * g == u_ini, ...
                       Yp * g == y_ini, ...
                       Uf * g == u, ...
                       Yf * g == y
                       ];
    elseif lower(constr_type) == 'e'
        constraints = [];
    end

    if lower(optimizer_type) == 'q'
        options = sdpsettings('solver', 'quadprog', 'verbose', 1);
    elseif lower(optimizer_type) == 'f'
        options = sdpsettings('solver', 'fmincon', 'verbose', 0);
        %options = sdpsettings('solver', 'fmincon', ...
        %                    'relax', 0, ...
        %                    'warning', 0, ...
        %                    'showprogress', 1 ...
        %                    );
    elseif lower(optimizer_type) == 'o'
        options = sdpsettings('solver', 'OSQP', ... % Use OSQP solver
                      'verbose', 0, ...             % Suppress solver output
                      'osqp.max_iter', 30000, ...   % Set maximum iterations
                      'osqp.eps_abs', 1e-7, ...     % Absolute tolerance
                      'warmstart', 0);             % Disable warm start
    else
        error('Error assigning solver options!');
    end
    
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