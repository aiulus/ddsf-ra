function conservatism_scores  = ddsfConservatism(filename)
    [descriptions, mode, sysname, sys, dims, T_sim, nruns] = name2params(filename);
    p = dims.p;
    
    plotdata = csv2plotdata(filename, mode, T_sim, dims);
    
    conservatism_scores = cell(1, nruns);

    for i=1:nruns
        sorted_i = sortData(plotdata, mode, i);
        y_i = sorted_i.y; yl_i = sorted_i.yl;
        c_i = zeros(1, p);
        for d=1:p
            yi_d = y_i(:, d, :); yli_d = yl_i(:, d, :);
            yi_d = yi_d(:); yli_d = yli_d(:);
            ci_d = conservatism(yi_d, yli_d);
            c_i(d) = ci_d;
        end
        c_i = mean(c_i);
        conservatism_scores{i} = c_i;
    end
end

