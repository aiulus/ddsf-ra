function gridPlot(mode, filename, m , p)
    [data, descriptions] = csvFlexRead(filename);

    nruns = numel(unique(descriptions));
    nfeatures = max(size(data));

    switch mode
        case 'ddsf'
            u_hist = cell2mat(data{1});
            ul_hist = cell2mat(data{2});
            for i=1:T_sim
                lower = (i-1)*m + 1; upper = i*m;
                u_i = u_hist(lower, upper);
                ul_i = ul_hist(lower, upper);
            end
        case 'deepc'
    end
end

function plotSingleRun(mode, data)
end

