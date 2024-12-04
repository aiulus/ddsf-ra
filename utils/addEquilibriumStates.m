function sys = addEquilibriumStates(sys)
    x_eq = sys.params.target;
    u_eq = -pinv(sys.B) * sys.A * x_eq;
    sys.S_f = struct( ...
        'x_eq', x_eq, ...
        'u_eq', u_eq);
end

