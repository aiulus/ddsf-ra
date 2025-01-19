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

function plotdata = getDataDeePC(data, m, p, T_sim)
    % TODO: Hash with configs
    nruns = size(data,1);
    t = 1:T_sim;
    plotU = {}; plotY = {};
    for i=1:nruns
        row_i = data(i, :);
        for ud=1:m
            u_indices_d = ud + (t-1)*m;            
            plotU{end} = u_indices_d;
        end
        for yd=1:p
            y_indices_d = yd + (t-1)*p + T_sim*m;  
            plotY{end} = y_indices_d;
        end
    end
    plotdata = struct('u', plotU, 'y', plotY);
end

function plotdata = getDataDDSF(m, T_sim)
    nruns = size(data,1);
    t = 1:T_sim;
    plotdata = {}; 
    for i=1:nruns
        row_i = data(i, :);
        for ud=1:m
            u_indices_d = ud + (t-1)*m;          
            ul_indices_d = uld + (t-1)*m + T_sim*m;  
            plotdata{end} = [u_indices_d,  ul_indices_d];
        end
    end

end