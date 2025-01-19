function batchPlot(mode, filename, m , p)
    [data, descriptions] = csvFlexRead(filename);
    
    T_sim = size(data, 2) / (m + p);
    nruns = numel(unique(descriptions));

    switch mode
        case 'ddsf'
            plotdata = getDataDDSF(data, descriptions, m, T_sim);
            plotU = plotdata.u; plotUL = plotdata.ul;
            for i=1:nruns
                prefix = string(descriptions(i));
                suffix = sprintf('-run%d', i);
                filename = strcat(prefix, suffix);
                u_vals_i = []; ul_vals_i = [];
                lower = (i -1)*T_sim + 1; upper = i*T_sim;
                for d=1:m                    
                    u_i_d = plotU(d, lower:upper);
                    u_vals_i = [u_vals_i; u_i_d];
                    ul_i_d = plotUL(d, lower:upper);
                    ul_vals_i = [ul_vals_i; ul_i_d];
                end
                gridplot(u_vals_i, ul_vals_i, filename);
            end
        case 'deepc'
            plotdata = getDataDeePC(data, configs, m, p, T_sim);
            plotU = plotdata.u; plotY = plotdata.y;
            for i=1:nruns
                prefix = string(descriptions(i));
                suffix = sprintf('-run%d', i);
                filename = strcat(prefix, suffix);
                u_vals_i = []; y_vals_i = [];
                lower = (i -1)*T_sim + 1; upper = i*T_sim;
                for ud=1:m                    
                    u_i_d = plotU(ud, lower:upper);
                    u_vals_i = [u_vals_i; u_i_d];                   
                end
                for yd=1:p                    
                    y_i_d = plotY(yd, lower:upper);
                    y_vals_i = [y_vals_i; y_i_d];
                end
                gridplot(u_vals_i, y_vals_i, filename);
            end
    end
end

function gridPlot(mode, vals1, vals2, filename)
    switch mode
        case 'ddsf'
            
        case 'deepc'
    end
end

function plotdata = getDataDeePC(data, configs, m, p, T_sim)
    % TODO: Hash with configs
    nruns = size(data,1);
    t = 1:T_sim;
    plotU = cell(m, nruns); plotY = cell(p, nruns); 
    for i=1:nruns
        row_i = data(i, :);
        row_i_arr = table2array(row_i);
        for ud=1:m
            u_indices_d = ud + (t-1)*m;        
            u_i = row_i_arr(u_indices_d);
            plotU{ud, i} = u_i;            
        end        
        for yd=1:p
            y_indices_d = yd + (t-1)*p + T_sim*m;  
            y_i = row_i_arr(y_indices_d);
            plotY{yd, i} = y_i;            
        end        
    end
    plotU = reshape(cell2mat(plotU), T_sim, []).';
    plotY = reshape(cell2mat(plotY), T_sim, []).';
    plotdata = struct('u', plotU, 'y', plotY, 'configurations', configs.');
end

function plotdata = getDataDDSF(data, configs, m, T_sim)
    nruns = size(data,1);
    t = 1:T_sim;
    plotU = cell(1, nruns); plotUL = cell(1, nruns); 
    for i=1:nruns
        row_i = data(i, :);
        row_i_arr = table2array(row_i);
        for ud=1:m
            u_indices_d = ud + (t-1)*m; 
            u_i = row_i_arr(u_indices_d);
            ul_indices_d = uld + (t-1)*m + T_sim*m;  
            ul_i = row_i_arr(ul_indices_d);
            plotU{i} = u_i; plotUL{i} = ul_i;  
        end
    end
    plotU = reshape(cell2mat(plotU), T_sim, []).';
    plotUL = reshape(cell2mat(plotUL), T_sim, []).';
    plotdata = struct('u', plotU, 'ul', plotUL, 'configurations', configs.');
end