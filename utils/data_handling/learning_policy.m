%   Can later be changed to:
%       u_l = learning_policy(y, y_d)
%
%   INPUTS:
%       y   - Current system output (px1)
%       y_d - Desired system output    

function u_l = learning_policy(lookup)
    sys = lookup.sys;
    m = lookup.sys.dims.m;
    % L = lookup.config.N + 2 * lookup.config.T_ini;
    Np = lookup.config.N;
    mode = lookup.data_options.datagen_mode;
    
    lb = sys.constraints.U(:, 1);
    lb(lb == -inf) = 1;
    ub = sys.constraints.U(:, 2);
    ub(ub == inf) = 1;
    
    switch mode
        case 'scaled_gaussian'
            scale = 1.25;
            ub = (ub > 0) .* (scale .* ub) + (ub < 0) .* ((scale^(-1)) .* ub) + (ub == 0) .* (10^scale);
            lb = (lb < 0) .* (scale .* lb) + (lb > 0) .* ((scale^(-1)) .* lb) + (lb == 0) .* (- 10^scale);
            ub = (ub > 0) .* (scale .* ub) + (ub < 0) .* ((scale^(-1)) .* ub);
            lb = (lb < 0) .* (scale .* lb) + (lb > 0) .* ((scale^(-1)) .* lb);
            u_l = lb + (ub - lb) .* rand(m, Np);
            % DEBUG STATEMENT
            % u_l = ones(m, L);
            %fprintf("2. <learning_policy> Relaxed lower and Upper bounds: [%d, %d]\n", lb, ub);
        case 'rbs'
            u_l = idinput([m, Np], 'rbs', [0, 1], [-1,1]);
        case 'scaled_rbs'
            lower = 0.5;
            upper = 0.8;
            num = 10;
            probs = (1/num) * ones(1, num);
            factors = linspace(lower, upper, num);
            scaler = randsample(factors, Np, true, probs);
            lb = lb .* scaler;
            ub = ub .* scaler;
    
            u_l = idinput([m, Np], 'rbs', [0, 1], [-1,1]);
            u_l = u_l .* (lb + (ub - lb) .* rand(1));
    end
end

    % Should this use the same policy as data generation?