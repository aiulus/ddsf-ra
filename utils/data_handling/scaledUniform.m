function signal = scaledUniform(factor, lb, ub, d1, d2)
    delta = (factor - 1) / 2;
    relaxed_lower = lb - delta * abs(lb);
    relaxed_upper = ub + delta * abs(ub);    
    signal = relaxed_lower + (relaxed_upper - relaxed_lower) .* rand(d1, d2);
end