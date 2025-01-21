function gridPlotDDSF(mode, configname, sys, sorted)
    switch mode
        case {'u-ddsf', 'ddsf'}
            u_hist = sorted.u; ul_hist = sorted.ul; time = 0:size(u_hist, 2) - 1;
            output_dir = prepareOutputDir();
            figure; tiledlayout(size(u_hist, 1), 1);
        
            for i = 1:size(u_hist, 1)
                nexttile;
                stairs(0:max(size(ul_hist(i, :)))-1, ul_hist(i, :), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
                hold on;
                stairs(0:max(size(u_hist(i, :)))-1, u_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));

                fprintf('gridPlotDDSF.m >> TIME STEP %d / %d Passing constraint U as: ', i, size(u_hist, 1)); disp(sys.constraints.U(i, :));
                fprintf('gridPlotDDSF.m >> TIME STEP %d / %d Passing config as: ', i, size(u_hist, 1)); disp(configname);
                addBounds(time, sys.constraints.U(i, :), configname);
               
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('u[%d]', i)); grid on; legend show;
                hold off;
            end
            saveAndClose(output_dir, configname);
        case 'y-ddsf'
            y_hist = sorted.y; yl_hist = sorted.yl; time = 0:size(y_hist, 2) - 1;
            output_dir = prepareOutputDir();
            figure; tiledlayout(size(y_hist, 1), 1);
        
            for i = 1:size(y_hist, 1)
                nexttile;
                stairs(0:max(size(yl_hist(i, :)))-1, yl_hist(i, :), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('ul[%d]', i));
                hold on;
                stairs(0:max(size(y_hist(i, :)))-1, y_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('u[%d]', i));
                addBounds(time, sys.constraints.Y(i, :), configname);
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('u[%d]', i)); grid on; legend show;
                hold off;
            end
            saveAndClose(output_dir, configname);
    end
end