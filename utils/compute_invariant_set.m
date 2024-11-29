function S_f = compute_invariant_set(sys)
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;
    U = sys.constraints.U;
    Y = sys.constraints.Y;

    max_iter = 100;
    tol = 1e-3;
    S_f = Y;
    for i = 1:max_iter
        S_f_next = PreImage(S_f, A, B, C, D, U);
        if S_f_next <= S_f % Convergence
            break;
        end
        S_f = S_f_next;
    end
end

