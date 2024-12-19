function [u_d, y_d, x_d, u, y] = ddsfGenerateData(lookup)
    %% Extract parameters
    sys = lookup.sys;
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    m = lookup.dims.m;
    n = lookup.dims.n;
    p = lookup.dims.p;

    T = lookup.config.T;

    mode = lookup.datagen_mode;

    %% Generate a random control input

    switch mode
        case 'rbs'
            PE_input = idinput([T, m], 'rbs', [0, 1], [-1,1]).'; 
        case 'scaled_gaussian'
            ub = sys.constraints.U(:, 2);
            ub(ub == inf) = 1;
            lb = sys.constraints.U(:, 1);
            lb(lb == -inf) = 1;

            PE_input = idinput([m, T], 'rbs', [0, 1], [0, 1]);
            for i = 1:m
                PE_input(:, i) = lb(i) + (ub(i) - lb(i)) * PE_input(:, i);
            end
       
    end
    

    % Initialize input-output storage
    u_d = zeros(m, T);
    y_d = zeros(p, T);
    x_d = zeros(n, T);

    % Generate data by simulating the system on random inputs for L steps
    for i = 1:T
        u_d(:, i) = PE_input(:, i);
        x_d(:, i + 1) = A * x_d(:, i) + B * u_d(:, i);
        y_d(:, i) = C * x_d(:, i) + D * u_d(:, i);
    end

    % Flatten the control inputs and outputs
    u = reshape(u_d, [], 1); % Reshapes into (T * sys.params.m) x 1
    y = reshape(y_d, [], 1); % Reshapes into (T * sys.params.p) x 1

end

