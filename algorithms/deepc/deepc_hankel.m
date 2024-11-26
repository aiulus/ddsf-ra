%% Creates Hankel matrices according to the Data Collection & Setup Process described in DeePC
function [Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, T_ini, N, sys)
    %   u_d: input data - vector/matrix with columns as inputs
    %   y_d: output data - vector/matrix with columns as outputs
    %   T_ini: #rows for past data
    %   N: prediction horizon
    
    [~, H_u] = construct_hankel(u_d, T_ini + N);
    [~, H_y] = construct_hankel(y_d, T_ini + N);

    full_rank = PEness_check(H_u);
    isPE = deepc_PEness_check(u_d, T_ini, N, sys);
    isPE = isPE & full_rank;

    if ~isPE
        error('Persistency of excitation check failed. Please provide richer input data or adjust T_ini and N.');
    end
    
    % Partition the Hankel matrices into past & future components
    Up = H_u(1:T_ini, :); % Past inputs
    Uf = H_u(T_ini + 1:end, :); % Future inputs
    Yp = H_y(1:T_ini, :); % Past outputs
    Yf = H_y(T_ini + 1:end, :); % Future outputs
end

