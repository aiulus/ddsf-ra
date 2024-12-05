function [x_next, y_next] = iterate_system(x, u, sys)
    % Extract state-space matrices
    A = sys.A;
    B = sys.B;
    C = sys.C;
    D = sys.D;

    % Apply the state-space matrices
    x_next = A * x + B * u.';
    y_next = C * x + D * u.';
end

