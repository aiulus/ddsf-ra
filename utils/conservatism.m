% DDSF - use with (lookup.logs.u, lookup.sys.constraints.U(1), lookup.sys.constraints.U(2))
function c = conservatism(u_safe, u_min, u_max)
    m = max(size(u_min));
    T_sim = size(u_safe, 2);
    if m ~= max(size(u_max))
        error('Lower and upper bounds must have the same shape!');
    end
    if size(u_safe, 1) ~= m
        error(['Input sequence must have the same input dimensionality as' ...
            ' u_min, u_max.']);
    end
    cd = zeros(m);
    for d=1:m
        maxrange = (u_max(d) - u_min(d))*T_sim;
        covered_range = sum(abs(u_safe(d, :)));
        %covered_range = sum(u_safe(d, :).^2);
        cd(d) = maxrange / covered_range;
    end
    c = mean(cd);
end