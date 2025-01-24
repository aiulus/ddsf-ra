function gridPlotDDSF(mode, configname, sys, sorted)
    switch mode
        case {'u-ddsf', 'ddsf'}
            u_hist = sorted.u; ul_hist = sorted.ul;
            T_sim = size(sorted.u, 3); time = 0:T_sim-1;
            m = sys.dims.m;
    
            output_dir = prepareOutputDir();
    
            plots_per_figure = 3; % Max number of plots per figure
            num_figures = ceil(m / plots_per_figure); % Number of figures needed
    
            for fig = 1:num_figures
                figure('Position', [100, 100, 800, 600]); % Fixed figure size
                tiledlayout(plots_per_figure, 1); % Max 3 subplots per figure
    
                for sub = 1:plots_per_figure
                    i = (fig - 1) * plots_per_figure + sub; % Global index for subplot
                    if i > m, break; end % Stop if no more subplots are needed
    
                    nexttile;
                    u_i_hist = u_hist(:, i, :);
                    ul_i_hist = ul_hist(:, i, :);
                    hold on;
    
                    % Plot the control input histories
                    stairs(time, squeeze(u_i_hist), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
                    stairs(time, squeeze(ul_i_hist), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
    
                    % Add bounds and grid
                    addBounds(time, sys.constraints.U(i, :), configname);
                    hold off;
    
                    % Configure plot titles and labels
                    title(sprintf('Control Input %d', i));
                    xlabel('t');
                    ylabel(sprintf('u[%d]', i));
                    grid on;
                    legend show;
                end
                saveAndClose(output_dir, sprintf('%s_fig%d', configname, fig));
            end
        case 'y-ddsf'
            y_hist = sorted.y; yl_hist = sorted.yl;
            T_sim = size(sorted.y, 3); time = 0:T_sim-1;
            p = sys.dims.p;
    
            output_dir = prepareOutputDir();
            plots_per_figure = 3; % Max number of plots per figure
            num_figures = ceil(p / plots_per_figure);
    
            for fig = 1:num_figures
                figure('Position', [100, 100, 800, 600]); % Set figure size
                tiledlayout(plots_per_figure, 1); % Max plots per figure
    
                for sub = 1:plots_per_figure
                    i = (fig - 1) * plots_per_figure + sub; % Global plot index
                    if i > p, break; end % Stop if we exceed the total number of plots
    
                    nexttile; hold on;
                    y_i_hist = y_hist(:, i, :); yl_i_hist = yl_hist(:, i, :);
    
                    stairs(time, squeeze(y_i_hist), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
                    stairs(time, squeeze(yl_i_hist), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('yl[%d]', i));
    
                    addBounds(time, sys.constraints.Y(i, :), configname);
                    hold off;
                    title(sprintf('Output %d', i)); xlabel('t'); ylabel(sprintf('y[%d]', i)); grid on; legend show;
                end
                saveAndClose(output_dir, printf('%s_fig%d', configname, fig));
            end
    
        case 'UUU-ddsf'
            u_hist = sorted.u; ul_hist = sorted.ul;
            T_sim = size(sorted.u, 3); time = 0:T_sim-1;
            m = sys.dims.m;
    
            output_dir = prepareOutputDir();
    
            figure; tiledlayout(m, 1);
    
            for i = 1:m
                nexttile;
                u_i_hist = u_hist(:, i, :); ul_i_hist = ul_hist(:, i, :);
                hold on;
                stairs(time, squeeze(u_i_hist), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
                stairs(time, squeeze(ul_i_hist), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
    
                addBounds(time, sys.constraints.U(i, :), configname);
                hold off;
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('u[%d]', i)); grid on; legend show;
            end
            saveAndClose(output_dir, configname);
        case 'YYY-ddsf'
            y_hist = sorted.y; yl_hist = sorted.yl;
            T_sim = size(sorted.y, 3); time = 0:T_sim-1;
            p = sys.dims.p;
            output_dir = prepareOutputDir();
    
            figure; tiledlayout(p, 1);
    
            for i = 1:p
                nexttile; hold on;
                y_i_hist = y_hist(:, i, :); yl_i_hist = yl_hist(:, i, :);
    
                stairs(time, squeeze(y_i_hist), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
                stairs(time, squeeze(yl_i_hist), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('yl[%d]', i));
    
                addBounds(time, sys.constraints.Y(i, :), configname);
                hold off;
                hold(ax, 'off');
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('y[%d]', i)); grid on; legend show;
            end
            saveAndClose(output_dir, configname);
    end
end