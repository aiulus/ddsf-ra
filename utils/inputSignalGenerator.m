function u = inputSignalGenerator(lookup, length)
    % Generates input signals within (relaxed) system constraints.
    %
    % Parameters:
    %   sys               - System structure containing constraints and dimensions.
    %   mode              - String specifying the input signal type:
    %                       'rbs'            : Random Binary Signal
    %                       'scaled_rbs'     : Scaled Random Binary Signal
    %                       'scaled_gaussian': Scaled Gaussian Signal
    %   length             - Integer specifying the number of time steps.
    %   relaxation_factor - Scalar to relax constraints (e.g., 1.25 for 25% relaxation).
    %
    % Returns:
    %   u                 - Generated input signal matrix of size (m x length).

    sys = lookup.sys; 
    mode = lookup.data_options.datagen_mode;
    scale = lookup.data_options.scale;

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
    u = zeros(m, length);

    % Generate input signals based on the specified mode
    switch mode
        case 'rbs'
            % Random Binary Signal
            u = idinput([length, m], 'rbs', [0, 1], [-1, 1])';
        case 'scaled_sinusoid'
            u = customSinusoid(sys, length, scale, 1e2);
        case 'scaled_gaussian'
            % Scaled Gaussian Signal
            for i = 1:m
                u(i, :) = lb(i) + (ub(i) - lb(i)) .* randn(1, length);
            end        
        case 'gendata_ddsf'
            ub = sys.constraints.U(:, 2);
            ub(ub == inf) = 1;
            lb = sys.constraints.U(:, 1);
            lb(lb == -inf) = 0;

            PE_input = idinput([m, length], 'rbs', [0, 1], [0, 1]);
            for i = 1:m
                PE_input(:, i) = lb(i) + (ub(i) - lb(i)) * PE_input(:, i);
            end
        case 'scaled_rbs'            
            lower = 0.5;
            upper = 0.8;
            num = 10;
            probs = (1/num) * ones(1, num);
            factors = linspace(lower, upper, num);
            scaler = randsample(factors, length, true, probs);
            lb = lb .* scaler;
            ub = ub .* scaler;

            u = idinput([m, length], 'rbs', [0, 1], [-1,1]); 
            u = u .* (lb + (ub - lb) .* rand(1));
        otherwise
            error(['Unsupported mode. Choose from ''rbs'',' ...
                ' ''scaled_rbs'', or ''scaled_gaussian''.']);
    end

    % Ensure signals are within relaxed bounds
    if lookup.data_options.safe
        u = max(min(u, ub), lb);
    end
end
