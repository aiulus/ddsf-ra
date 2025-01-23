function [u_d, y_d, x_d, u, y] = genDataNonlinear(lookup)
    %% Extract parameters
    sys = lookup.sys;
    Fx_eval = sys.Fx_eval; % Nonlinear dynamics (cell of function handles)
    Gx_eval = sys.Gx_eval; % Nonlinear measurements (cell of function handles)

    params = sys.params;
    T = lookup.config.T;

    % State and input dimensions
    n = length(params.x_ini); % State dimension
    m = 1; % Assuming scalar input (adjust as needed)
    p = length(Gx_eval); % Number of outputs

    %% Generate a random control input
    PE_input = inputSignalGenerator(lookup, T);

    % Initialize input-output storage
    u_d = zeros(m, T);
    y_d = zeros(p, T);
    x_d = zeros(n, T + 1);

    % Set initial state
    x_d(:, 1) = params.x_ini;

    %% Simulate the system
    for t = 1:T
        u_d(:, t) = PE_input(:, t); % Input at time t

        % Evaluate dynamics
        f_eval = cellfun(@(f) f(x_d(:, t), u_d(:, t)), Fx_eval, 'UniformOutput', false);
        x_d(:, t + 1) = cell2mat(f_eval);

        % Evaluate measurements
        g_eval = cellfun(@(g) g(x_d(:, t), u_d(:, t)), Gx_eval, 'UniformOutput', false);
        y_d(:, t) = cell2mat(g_eval);
    end

    % Flatten the control inputs and outputs
    u = reshape(u_d, [], 1); % Reshapes into (T * m) x 1
    y = reshape(y_d, [], 1); % Reshapes into (T * p) x 1
end
