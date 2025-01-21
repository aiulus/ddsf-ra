function u = randomInputGenerator(lookup)
    % Parameter extraction
    sys = lookup.sys;
    dims = lookup.dims;
    m = dims.m; n = dims.n; p = dims.p;
    T = lookup.config.T;

    mode = lookup.data_options.datagen_mode;
    scale = lookup.data_options.scale;    

    lb = sys.constraints.U(:, 1);
    ub = sys.constraints.U(:, 2);
    [lb, ub] = handleBounds(lb, ub, scale);
    u = zeros(m, T);

    switch mode
        case 'gaussian'
            t = rand(m, T);
            for i=1:T
                % ACHTUNG: Incredibly large values for scale = inf
                u(:, i) = (ub - lb).*t(:, i) + lb;
                if (ub - lb) >= 1e+8
                    u(:,  i) = log(u(:, i));
                end
            end
        case 'pseudo_binary'
            u = idinput([m, T], 'prbs', [0, 1], [-1,1]); 
    end
end

function [lower, upper] = handleBounds(lb, ub, scale)
    inf_subst = 1e+8;

    lower = lb; 
    upper = ub; 

    if lb == -inf || scale == inf
        lower = -inf_subst;
    end
    if ub == inf || scale == inf
        upper = inf_subst;
    end

    if scale > 1
        lower = lower - abs(lower) / scale;
        upper = upper + abs(upper) / scale;
    end
end

