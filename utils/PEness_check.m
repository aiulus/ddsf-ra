function isPE= PEness_check(H, u_d, T_ini, N, sys)
%   PEness_check Verifies if the Hankel matrix H has full rank
    r = rank(H);
    d = min(size(H));
    if r == d
        isPE = true;
        disp('Hankel matrix has full rank.')
    else
        isPE = false;
        disp("Hankel matrix doesn't have full rank!");
    end

    m = size(u_d, 1); % Input dimension
    n_B = size(sys.A, 1); % Minimum repr. dim.
    required = (m + 1) * (T_ini + N + n_B) - 1; % Minimum required data length

    if size(u_d, 2) < required
        fprintf('Data length T=%d is insufficient. Required: %d\n', size(u_d, 2), required);
        isPE = false;
    else
        disp('Data length satisfies persistency of excitation condition.');
        isPE = true;
    end
end

