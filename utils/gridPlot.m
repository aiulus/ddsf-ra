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
    %t = 1:T_sim;
    plotU = cell(1, nruns); plotY = cell(1, nruns);
    for i=1:nruns
        row_i = data(i, :);
        for ud=1:m
            % u_indices_d = ud + (t-1)*m;            
            % plotU{i} = row_i(u_indices_d);            
        end
        plotU{i} = reshape(row_i(reshape((0:(T_sim-1))' * m, [], 1) + (1:m)), T_sim, m)';
        for yd=1:p
            % y_indices_d = yd + (t-1)*p + T_sim*m;  
            % plotY{i} = row_i(y_indices_d);            
        end
        plotY{i} = reshape(row_i(T_sim * m + reshape((0:(T_sim-1))' * p, [], 1) + (1:p)), T_sim, p)';
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
            plotdata{end} = [row_i(u_indices_d),  row_i(ul_indices_d]);
        end
    end

end