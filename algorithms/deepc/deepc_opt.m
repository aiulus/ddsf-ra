%% Defines and solves an optimization problem as described in DeePC

function [g_opt, u_opt, y_opt] = deepc_opt(Up, Yp, Uf, Yf, u_ini, y_ini, r, Q, R, U, Y, N, T)
    T_ini = max(size(u_ini));
    g = sdpvar(size(Uf, 2), 1); % Canocical ptimization variable € R^{(T - T_{ini} - N + 1) x 1}

    disp("Size of g: "); 
    fprintf("[%d, %d] \n", size(g, 1), size(g, 2));
    disp("U_f of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(Uf, 1), size(Uf, 2)); % DEBUG STATEMENT
    disp(Uf), % DEBUG STATEMENT     
    disp("Y_f of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(Yf, 1), size(Yf, 2)); % DEBUG STATEMENT
    disp(Yf), % DEBUG STATEMENT
    disp("U_p of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(Up, 1), size(Up, 2)); % DEBUG STATEMENT
    disp(Up), % DEBUG STATEMENT     
    disp("Y_p of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(Yp, 1), size(Yp, 2)); % DEBUG STATEMENT
    disp(Yp), % DEBUG STATEMENT
    disp("u_ini of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(u_ini, 1), size(u_ini, 2)); % DEBUG STATEMENT
    disp(u_ini), % DEBUG STATEMENT     
    disp("y_ini of order: "); % DEBUG STATEMENT
    fprintf("[%d, %d]", size(y_ini, 1), size(y_ini, 2)); % DEBUG STATEMENT
    disp(y_ini), % DEBUG STATEMENT

    u = Uf * g; % Predicted inputs
    y = Yf * g; % Predicted outputs 

    % Display debug information
    % disp("size of Up: "); disp(size(Up));
    % disp("Up: "); disp(Up);
    % disp("size of Yp: "); disp(size(Yp));
    % disp("Yp: "); disp(Yp);
    % disp("size of Uf: "); disp(size(Uf));
    % disp("Uf: "); disp(Uf);
    % disp("size of Yf: "); disp(size(Yf));
    % disp("Yf: "); disp(Yf);
    % disp("size of g: "); disp(size(g));
    % disp("size of u: "); disp(size(u));
    % disp("size of U: "); disp(size(U));
    % disp("U: "); disp(U);
    % disp("size of y: "); disp(size(y));
    % disp("size of Y: "); disp(size(Y));
    % disp("Y: "); disp(Y);
    % disp("size of u_ini: "); disp(size(u_ini));


    % Define the cost function
    cost = 0;
    for k = 1:N
        cost = cost + (y(k) - r(k))' * Q * (y(k) - r(k)) + u(k)' * R * u(k);
    end
    
    % Define the constraints
    % constraints = [Up * g == u_ini, Yp * g == y_ini, u <= U, y <= Y];
    % constraints = [Up * g == u_ini, Yp * g == y_ini];

    constraints = [Up * g == u_ini, Yp * g == y_ini, ...,
                  %  Uf * g == u, Yf * g == y,
               u >= U(1), u <= U(2), ...
               y >= Y(1), y <= Y(2)];


    % Solve optimization problem
    options = sdpsettings('verbose', 0, 'solver', 'quadprog');
    diagnostics = optimize(constraints, cost, options); 

    %if diagnostics.problem == 0 % Feasible solution found
    g_opt = value(g);
    u_opt = value(u);
    y_opt = value(y);
    % fprintf("(OPTIMIZER) g_opt=%, u_opt=%, y_opt=%", g_opt, u_opt, y_opt);
    fprintf("OPTIMIZER running..."); % DEBUG STATEMENT
    %else
    %    disp("Constraints:");  disp(constraints);
    %    disp("Cost function:"); disp(cost);        
    %    error('Optimization problem is infeasible!')
    % end
end

