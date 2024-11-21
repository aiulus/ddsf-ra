%% Creates Hankel matrices according to the Data Collection & Setup Process described in DeePC
function [Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, T_ini, N)
    %   u_d: input data - vector/matrix with columns as inputs
    %   y_d: output data - vector/matrix with columns as outputs
    %   T_ini: #rows for past data
    %   N: prediction horizon
    
    H_u = custom_hankel(u_d, T_ini + N);
    H_y = custom_hankel(y_d, T_ini + N);
    
    % Partition the Hankel matrices into past & future components
    Up = H_u(1:T_ini, :); % Past inputs
    Uf = H_u(T_ini + 1:end, :); % Future inputs
    Yp = H_y(1:T_ini, :); % Past outputs
    Yf = H_y(T_ini + 1:end, :); % Future outputs
end

