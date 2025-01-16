function u = inputSignalGenerator(sys, mode, T_sim, scale)
    % Generates input signals within (relaxed) system constraints.
    %
    % Parameters:
    %   sys               - System structure containing constraints and dimensions.
    %   mode              - String specifying the input signal type:
    %                       'rbs'            : Random Binary Signal
    %                       'scaled_rbs'     : Scaled Random Binary Signal
    %                       'scaled_gaussian': Scaled Gaussian Signal
    %   T_sim             - Integer specifying the number of time steps.
    %   relaxation_factor - Scalar to relax constraints (e.g., 1.25 for 25% relaxation).
    %
    % Returns:
    %   u                 - Generated input signal matrix of size (m x T_sim).

    % Extract system dimensions and constraints
    m = sys.dims.m;
    lb = sys.constraints.U(:, 1);
    ub = sys.constraints.U(:, 2);

    % Replace infinite bounds with finite values
    lb(lb == -inf) = -1;
    ub(ub == inf) = 1;

    % Apply relaxation factor to constraints
    lb = lb * scale;
    ub = ub * scale;

    % Initialize input signal matrix
    u = zeros(m, T_sim);

    % Generate input signals based on the specified mode
    switch mode
        case 'rbs'
            % Random Binary Signal
            u = idinput([T_sim, m], 'rbs', [0, 1], [-1, 1])';
        case 'scaled_sinusoid'
            u = customSinusoid(sys, T_sim, scale, 1e2);
        case 'scaled_gaussian'
            % Scaled Gaussian Signal
            for i = 1:m
                u(i, :) = lb(i) + (ub(i) - lb(i)) .* randn(1, T_sim);
            end

        otherwise
            error('Unsupported mode. Choose from ''rbs'', ''scaled_rbs'', or ''scaled_gaussian''.');
    end

    % Ensure signals are within relaxed bounds
    u = max(min(u, ub), lb);
end
