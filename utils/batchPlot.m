function batchPlot(mode, filename, m , p)
    [data, descriptions] = csvFlexRead(filename);
    
    T_sim = size(data, 2) / (m + p);
    nruns = numel(unique(descriptions));
    
    switch mode
        case 'ddsf'
            plotdata = getDataDDSF(data, descriptions, m, T_sim);       
                
            if filename(1) == 'U'
                plotU = plotdata.u; plotUL = plotdata.ul;
                for i=1:nruns
                    prefix = string(descriptions(i));
                    suffix = sprintf('-run%d', i);
                    config = strcat(prefix, suffix);
                    u_vals_i = []; ul_vals_i = [];
                    lower = (i -1)*T_sim + 1; upper = i*T_sim;
                    for d=1:m
                        u_i_d = plotU(d, lower:upper);
                        u_vals_i = [u_vals_i; u_i_d];
                        ul_i_d = plotUL(d, lower:upper);
                        ul_vals_i = [ul_vals_i; ul_i_d];
                    end
                    gridPlot(u_vals_i, ul_vals_i, config);
                end
            end
            if filename(1) == 'Y'
                plotY = plotdata.u; plotYL = plotdata.yl;
                for i=1:nruns
                    prefix = string(descriptions(i));
                    suffix = sprintf('-run%d', i);
                    config = strcat(prefix, suffix);
                    y_vals_i = []; yl_vals_i = [];
                    lower = (i -1)*T_sim + 1; upper = i*T_sim;
                    for d=1:p
                        y_i_d = plotU(d, lower:upper);
                        y_vals_i = [y_vals_i; y_i_d];
                        yl_i_d = plotUL(d, lower:upper);
                        yl_vals_i = [yl_vals_i; yl_i_d];
                    end
                    gridPlot('ddsf', config, y_vals_i, yl_vals_i);
                end
            end
    
        case 'deepc'
            plotdata = getDataDeePC(data, descriptions, m, p, T_sim);
            plotU = plotdata.u; plotY = plotdata.y;
            for i=1:nruns
                prefix = string(descriptions(i));
                suffix = sprintf('-run%d', i);
                config = strcat(prefix, suffix);
                u_vals_i = []; y_vals_i = [];
                %lower = (i -1)*T_sim + 1; upper = i*T_sim;
                lower = 1; upper = T_sim;
                for ud=1:m
                    u_i_d = plotU(ud, lower:upper);
                    u_vals_i = [u_vals_i; u_i_d];
                end
                for yd=1:p
                    y_i_d = plotY(yd, lower:upper);
                    y_vals_i = [y_vals_i; y_i_d];
                end
                gridPlot('deepc', config, u_vals_i, y_vals_i);
            end
    end
end

function gridPlot(mode, filename, varargin)
    switch lower(mode)
        case 'ddsf'
            if nargin < 6
                error('For DDSF mode, you must provide: filename, u_hist, ul_hist, y_hist, yl_hist.');
            end
            
            u_hist = varargin{1};
            ul_hist = varargin{2};
            y_hist = varargin{3};
            yl_hist = varargin{4};
            
            gridPlotDDSF(filename, u_hist, ul_hist, y_hist, yl_hist);
            
        case 'deepc'
            if nargin < 4
                error('For DeePC mode, you must provide: filename, u_hist, y_hist.');
            end
            
            u_hist = varargin{1};
            y_hist = varargin{2};
            
            gridPlotDeePC(filename, u_hist, y_hist);
            
        otherwise
            error('Invalid mode. Supported modes are ''ddsf'' and ''deepc''.');
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
    plotY = cell(1, nruns); plotYL = cell(1, nruns); 
    for i=1:nruns
        row_i = data(i, :);
        row_i_arr = table2array(row_i);
        for ud=1:m
            u_indices_d = ud + (t-1)*m; 
            u_i = row_i_arr(u_indices_d);
            ul_indices_d = ud + (t-1)*m + T_sim*m;  
            ul_i = row_i_arr(ul_indices_d);
            plotU{i} = u_i; plotUL{i} = ul_i;  
        end
    end

    plotU = reshape(cell2mat(plotU), T_sim, []).';
    plotUL = reshape(cell2mat(plotUL), T_sim, []).';

    plotY = reshape(cell2mat(plotY), T_sim, []).';
    plotYL = reshape(cell2mat(plotYL), T_sim, []).';

    plotdata = struct('u', plotU, 'ul', plotUL, 'y', plotY, 'yl', plotYL, configurations', configs.');
end

function gridPlotDDSF(filename, val1, val2)
    sysname = filename2sysname(filename);
    sys = sysInit(sysname);
    plotmode = (filename(1) == 'U');
    
    if plotmode
        u_hist = val1;
        ul_hist = val2;
        time = 0:size(u_hist, 2) - 1;
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
        saveas(gcf, strcat(filename, '_inputs.png'));
        matlab2tikz(strcat(filename, '_inputs.tex'));
        close(gcf);
    else
        y_hist = val1;
        yl_hist = val2;
        time = 0:size(y_hist, 2) - 1;
    
        figure;
        p = size(y_hist, 1);
        tiledlayout(p, 1); % Combine all inputs into a single layout
    
        %% TODO
        for i = 1:p
            nexttile; hold on;
            plot(time, y_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
            plot(time, yl_hist(i, :), 'r', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
            bounds = sys.constraints.Y(i, :);
    
            % Plot boundaries
            if bounds(1) ~= -inf
                plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
            end
            if bounds(2) ~= inf
                plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
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
        saveas(gcf, strcat(filename, '_outputs.png'));
        matlab2tikz(strcat(filename, '_outputs.tex'));
        close(gcf);
    end
end

function gridPlotDeePC(filename, val1, val2)
    sysname = filename2sysname(filename);
    fprintf('Using sysname: %s', sysname);
    sys = sysInit(sysname);      

    u_hist = val1;
    y_hist = val2;

    time = 0:size(u_hist, 2) - 1;

    figure;
    p = size(y_hist, 1);
    % TODO: THAT WON'T BE CORRECT - CAN BE PARSED FROM THE FILE NAME
    sys.target = sys.params.target;

    tiledlayout(p, 1);

    for i=1:p
        subplot(p, 1, i);
        y_name = sprintf("y%d", i);
        hold on;
        plot(time, y_hist(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', y_name); 
        xlabel('Iteration #');
        ylabel(sprintf('Output %d', i));

        target = sys.target * ones(1, size(time, 2));
        plot(time, target, 'm--', 'DisplayName', 'Target');
       
        grid on; legend show; hold off;
    end
    sgtitle("Control inputs over time");
    saveas(gcf, strcat(filename, '_outputs.png'));
    %matlab2tikz(strcat(filename, '_outputs.tex'));
    close(gcf); % Close the figure to prevent display

    figure;
    m = size(u_hist, 1);
    tiledlayout(m, 1);

    for i=1:m
        subplot(m, 1, i);
        u_name = sprintf("u%d", i);
        hold on;
        stairs(time, u_hist(1, :), 'r', 'LineWidth', 1.25, 'DisplayName', u_name);
        xlabel('Iteration #');
        ylabel(sprintf('Control input %d', i));

        % Plot boundaries
        if bounds(1) ~= -inf
            plot(time, bounds(1) * ones(size(time)), 'm--', 'DisplayName', 'Lower Bound');
        end
        if bounds(2) ~= inf
            plot(time, bounds(2) * ones(size(time)), 'k--', 'DisplayName', 'Upper Bound');
        end
        grid on; legend show; hold off;
    end
    sgtitle("Control Inputs Over Time");
    saveas(gcf, strcat(filename, '_inputs.png'));
    %matlab2tikz(strcat(filename, '_inputs.tex'));
    close(gcf); 
end