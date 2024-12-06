function sys = addEquilibriumStates(sys)
    U = sys.constraints.U; % Input constraints set
    Y = sys.constraints.Y; % Output constraints set

    A = full(sys.A);
    B = full(sys.B);
    C = full(sys.C);
    D = full(sys.D);

    n = sys.params.n;

    % x_s = sdpvar(n, 1);

    T_u = pinv(B) * (eye(n) - A);
    T_y = C + D * T_u;
    
    % Compute the column space of T_u
    [U_eq, ~, ~] = svd(T_u, 'econ');
    [Y_eq, ~, ~] = svd(T_y, 'econ');

    U_s = intersect(U_eq, U);
    Y_s = intersect(Y_eq, Y);

    sys.S_f = struct( ...
        'U_s', U_s, ...
        'Y_s', Y_s ...
        );
end

