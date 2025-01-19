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

function gridPlotDDSF(filename, val1, val2, val3, val4)    
    sysname = filename2sysname(filename);
    sys = sysInit(sysname);    

    u_hist = val1;
    ul_hist = val2;
    time = 0:size(u_hist, 2) - 1;

    y_hist = val3;
    yl_hist = val4;
        
    figure;
    m = size(u_hist, 1);
    tiledlayout(m, 1); 
    
    % Plot learning vs safe inputs
    for i = 1:m
        nexttile;
        stairs(0:size(ul_hist, 2) - 1, ul_hist(i, :), 'r', 'LineStyle', ':','LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
        hold on;
        stairs(0:size(u_hist, 2) - 1, u_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
    
        bounds = sys.constraints.U(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
    
        title(sprintf('Learning vs Safe Input %d', i));
        xlabel('t');
        ylabel(sprintf('Input %d', i));
        grid on;
        legend show;
        hold off;
    end        
    sgtitle('Learning Inputs vs. Safe Inputs');
    saveas(gcf, strcat(base_filename, '_inputs.png'));
    matlab2tikz(strcat(base_filename, '_inputs.tex'));
    close(gcf); 

    figure;
    p = size(y_hist, 1);
    tiledlayout(p, 1); % Combine all inputs into a single layout

    %% TODO
    for i = 1:p
        nexttile; hold on;
        plot(time, y_hist(i, :), 'k', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
        bounds = sys.constraints.Y(i, :);
    
        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'b--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'r--', 'DisplayName', 'Upper Bound');
        end
        if lookup.opt_params.target_penalty
            pi = sys.params.target(i);
            plot(time, pi * ones(size(time)), 'g--', 'DisplayName', 'Target');
        end
    
        title(sprintf('System Output %d', i));
        xlabel('t');
        ylabel(sprintf('Output %d', i));
        grid on;
        legend show;
        hold off;
    end
    sgtitle("Resulting Outputs");
    saveas(gcf, strcat(base_filename, '_outputs.png'));
    matlab2tikz(strcat(base_filename, '_outputs.tex'));
    close(gcf); 
end

