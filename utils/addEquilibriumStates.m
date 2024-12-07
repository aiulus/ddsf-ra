function sys = addEquilibriumStates(sys)
    U = sys.constraints.U; % Input constraints set
    Y = sys.constraints.Y; % Output constraints set

    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    n = sys.params.n;
    k = min(size(B)); % Number of SVD values needed for the pseudo-inverse

    % x_s = sdpvar(n, 1);

    T_u = sparse_pinv(B, k) * (eye(n) - A);
    T_y = C + D * T_u;
    
    
    % Compute the column space of T_u
    %[U_eq, ~, ~] = svd(T_u, 'econ');
    %[Y_eq, ~, ~] = svd(T_y, 'econ');

    % Represent the safe sets with polytopes
    U_s = computeSafeSet(T_u, U);
    Y_s = computeSafeSet(T_y, Y);

    sys.S_f = struct( ...
        'U_s', U_s, ...
        'Y_s', Y_s ...
        );
end

function X_s = computeSafeSet(T_x, X)
    X_eq = polytope(T_x, zeros(size(T_x, 1), 1));
    X_s = intersect(X_eq, polytope(X(1), X(2)));
end


