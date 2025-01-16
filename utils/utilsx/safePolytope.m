function X_s = safePolytope(T_x, X)
    X_eq = polytope(T_x, zeros(size(T_x, 1), 1));
    I_x = interval(X(:, 1), X(:, 2));
    P_x = polytope(I_x);
    X_s = intersect(X_eq, P_x);
end