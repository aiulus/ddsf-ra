function y_pred = dataBasedU2Y(u_l, u_ini, y_ini, u_d, y_d)
    H_u = construct_hankel(u_d, 2);
    H_y = construct_hankel(y_d, 2);

    numcols = size(H_u, 2);    
    p = max(size(y_ini));

    if size(H_y, 2) ~= numcols
        error('The Hankel matrices H_u and H_y must have a matching number of columns!');
    end

    H = [H_u; H_y];

    alpha = sdpvar(numcols, 1);
    y_pred = sdpvar(p, 1);

    u = [u_ini; u_l];
    y = [y_ini; y_pred];
    traj = [u; y];

    constraints = traj == H * alpha;
    objective = alpha' * alpha;

    options = sdpsettings('solver', 'quadprog', 'verbose', 0);
    diagnostics = optimize(constraints, objective, options);
    
    if diagnostics.problem == 0
        % Extract the optimal prediction
        y_pred = value(y_pred);
    else
        error('Optimization problem could not be solved!');
    end
   
end

