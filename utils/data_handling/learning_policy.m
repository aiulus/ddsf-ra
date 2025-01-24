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
    scale = lookup.data_options.scale;
    
    lb = sys.constraints.U(:, 1);
    lb(lb == -inf) = 1;
    ub = sys.constraints.U(:, 2);
    ub(ub == inf) = 1;
    
    switch mode
        case 'custom_uniform'
            u_l = customUniform(scale, lb, ub, m, Np);
        case 'scaled_uniform'
            u_l = scaledUniform(scale, lb, ub, m, Np);
        case 'prbs'
            u_l = idinput([m, Np], 'prbs', [0, 1], [-1,1]);
        case 'sinusoid'
            u_l = customSinusoid(lookup.sys, lookup.T_sim, scale, 1e+8);
    end
end




