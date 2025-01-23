function sys = nonlinearSysInit(sys_type)
    switch sys_type
        case 'logmap'
            % TODO
            params = struct( ...
                'mu', 1, ...             % Nonlinearity parameter
                'x_ini', [2; 0], ...     % Initial state
                'u_min', -5, ...
                'u_max', 5 ...
            );
            
            % TODO

            % System configuration
            config = struct( ...
                'T', 50, ...              % Time horizon
                'T_ini', 10, ...          % Initial trajectory length
                'N', 15 ...               % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Van der Pol Oscillator
        case 'van_der_pol'
            params = struct( ...
                'mu', 1, ...             % Nonlinearity parameter
                'x_ini', [2; 0], ...     % Initial state
                'u_min', -5, ...
                'u_max', 5 ...
            );
        
            % TODO
           
                
            % System configuration
            config = struct( ...
                'T', 100, ...            % Time horizon
                'T_ini', 20, ...         % Initial trajectory length
                'N', 30 ...              % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        %% Nonlinear Pendulum
        case 'nonlinear_pendulum'
            params = struct( ...
                'g', 9.81, ...           % Gravity
                'l', 1, ...              % Length of pendulum
                'x_ini', [pi/4; 0], ...  % Initial angle and angular velocity
                'u_min', -2, ...
                'u_max', 2 ...
            );

            statevars = struct( ...
                'x1', sdpvar(1), ... % Angle
                'x2', sdpvar(1) ...  % Angular velocity
                );

            inputvars = struct( ...
                'u', sdpvar(1) ... % Force
                );

            f1 = @(x1, x2) x2;
            f2 = @(x1, x2) -r*x2 - (g/l)*sin(x1);
            g1 = @(x1, x2) x1;

            Fx = [f1; f2]; % Dynamics functions
            Gx = [g1]; % Measurement functions

            % System configuration
            config = struct( ...
                'T', 80, ...             % Time horizon
                'T_ini', 10, ...         % Initial trajectory length
                'N', 20 ...              % Prediction horizon
            );

            opt_params = struct('Q', 1, 'R', 1);

        otherwise
            error('Unknown system type specified.');
    end

    % Populate system with parameters and configuration
    [A, B, C, D] = linearize(Fx, Gx, statevars, inputvars);
    sys.A = A; sys.B = B; sys.C = C; sys.D = D;
    sys.config = config; sys.opt_params = opt_params;
end

function [A, B, C, D] = linearize(Fx, Gx, x, u)
    [x_e, u_e] = getEquilibrium(Fx, x, u);
    Asym = computeJacobian(Fx, x);
    Bsym = computeJacobian(Fx, u);
    Csym = computeJacobian(Gx, x);
    Dsym = computeJacobian(Gx, u);

    A = evaluateJacobian(Asym, x_e);
    B = evaluateJacobian(Bsym, u_e);
    C = evaluateJacobian(Csym, x_e);
    D = evaluateJacobian(Dsym, u_e);
end

function [x_e, u_e] = getEquilibrium(Fx, x, u)
    x_e = zeros(max(size(Fx)), 1); u_e = zeros(max(size(Gx)), 1);
    for i=1:max(size(Fx))
        fi = Fx(i);
        [xei, uei] = solve(fi, [x, u]); % Solve for fi(x, u) = 0 for all i 
        x_e(i) = xei; u_e(i) = uei;
    end
end

function Asym = computeJacobian(Fx, x)
    % Fx: A collection of N functions f: R^{N+m} --> 1
    N1 = max(size(Fx)); N2 = max(size(x));
    J = zeros(N1, N2);
    for i=1:N1
        fi = Fx(i);
        delta_fi = multivariate_derivative(fi, x); % Returns a 1xN - row vector
        J(i, :) = delta_fi; 
    end
end

function A = evaluateJacobian(Asym, x_e)
    A = zeros(size(Asym));
    for i=1:size(A, 1)
        fi = Asym(i, :);
        for j=1:size(A, 2)
            xej = x_e(j);
            dfi_dxj = fi(j);
            A(i, j) = subs(dfi_dxj, xj, xej);
            A(i, j) = value(A(i, j));
        end        
    end
end

function delta_fi = multivariate_derivative(fi, x)
    N = max(size(x));
    delta_fi = zeros(1, N);
    for j=1:N
        xj = x(j);
        dfi_dxj = derive(fi, xj);
        delta_fi(j) = dfi_dxj;
    end
end