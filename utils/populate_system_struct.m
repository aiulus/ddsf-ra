function sys = populate_system_struct(A, B, C, D, params)
    % Constructs the system struct with matrices, dimensions, and constraints.
    sys.A = A;
    sys.B = B;
    sys.C = C;
    sys.D = D;

    % Dimensions
    sys.dims.state = size(A, 1);
    sys.dims.input = size(B, 2);
    sys.dims.output = size(C, 1);

    % Constraints
    sys.constraints.U = [params.u_min, params.u_max];
    sys.constraints.Y = [params.y_min, params.y_max];
    %sys.constraints.U_poly = polyhe

    % Parameters and target
    sys.params = params;
    sys.target = params.target;
end