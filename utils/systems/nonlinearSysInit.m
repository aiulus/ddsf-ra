function sys = nonlinearSysInit(sys_type)
    % Initialize the system based on the specified type
    switch sys_type
        %% Nonlinear Test System (m=1, p=1, with delay)
        case 'test_nonlinear'
            params = struct( ...
                'x_ini', 0.5, ...    % Initial state
                'target', 1, ...     % Target state
                'u_min', -10, ...    % Min input
                'u_max', 10 ...      % Max input
            );

            % Define nonlinear dynamics and output functions
            f = @(x, u) -0.5 * x^2 + u;   % Nonlinear dynamics
            h = @(x) 2 * x;               % Nonlinear output

            % Incorporate input delay and discretization
            delay = 1;                    % Delay of 1 sample
            Ts = 0.1;                     % Sampling time
            [Ad, Bd, Cd, Dd] = c2d_nonlinear(f, h, Ts, delay);

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

            % Define nonlinear dynamics and output functions
            f = @(x, u) [x(2); params.mu * (1 - x(1)^2) * x(2) - x(1) + u];
            h = @(x) x(1);               % Output is the first state

            % Discretization
            Ts = 0.05;                   % Sampling time
            [Ad, Bd, Cd, Dd] = c2d_nonlinear(f, h, Ts, 0); % No delay

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

            % Define nonlinear dynamics and output functions
            f = @(x, u) [x(2); -(params.g / params.l) * sin(x(1)) + u];
            h = @(x) x(1);               % Output is the angle

            % Discretization
            Ts = 0.1;                    % Sampling time
            [Ad, Bd, Cd, Dd] = c2d_nonlinear(f, h, Ts, 0); % No delay

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
    sys = struct('A', Ad, 'B', Bd, 'C', Cd, 'D', Dd);
    sys = populate_system(sys, params, opt_params, config);
end

function [Ad, Bd, Cd, Dd] = c2d_nonlinear(f, h, Ts, delay)
    % Approximate discrete-time state-space matrices for a nonlinear system
    x_eq = 0;  % Equilibrium state
    u_eq = 0;  % Equilibrium input

    % Linearization at equilibrium
    A = numerical_jacobian(@(x) f(x, u_eq), x_eq);
    B = numerical_jacobian(@(u) f(x_eq, u), u_eq);
    C = numerical_jacobian(@(x) h(x), x_eq);
    D = 0;  % Assuming no feedthrough

    % Discretize using sampling time
    Ad = expm(A * Ts);
    Bd = integral_discretize(A, B, Ts, delay);
    Cd = C;
    Dd = D;
end

function J = numerical_jacobian(fun, x)
    % Compute numerical Jacobian
    delta = 1e-6;
    n = length(x);
    J = zeros(n, n);
    for i = 1:n
        x_perturbed = x;
        x_perturbed(i) = x_perturbed(i) + delta;
        J(:, i) = (fun(x_perturbed) - fun(x)) / delta;
    end
end

function Bd = integral_discretize(A, B, Ts, delay)
    % Discretize B matrix with delay
    Bd = expm(A * Ts) * B;  % Zero-order hold assumption
    if delay > 0
        Bd = [zeros(size(B, 1), delay), Bd];
    end
end
