function gridPlotDDSF(mode, configname, sys, sorted)
    switch mode
        case {'u-ddsf', 'ddsf'}             
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

                %fprintf('gridPlotDDSF.m >> TIME STEP %d / %d Passing constraint U as: ', i, size(u_hist, 1)); disp(sys.constraints.U(i, :));
                %fprintf('gridPlotDDSF.m >> TIME STEP %d / %d Passing config as: ', i, size(u_hist, 1)); disp(configname);
                addBounds(time, sys.constraints.U(i, :), configname);
                hold off;
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('u[%d]', i)); grid on; legend show;                
            end
            saveAndClose(output_dir, configname);
        case 'y-ddsf'
            y_hist = sorted.y; yl_hist = sorted.yl; 
            T_sim = size(sorted.y, 3); time = 0:T_sim-1;
            p = sys.dims.p;
            output_dir = prepareOutputDir();

            figure; tiledlayout(p, 1);
        
            for i = 1:size(y_hist, 1)
                nexttile; hold on;
                y_i_hist = y_hist(:, i, :); yl_i_hist = yl_hist(:, i, :);              
                
                stairs(time, squeeze(y_i_hist), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
                stairs(time, squeeze(yl_i_hist), 'r:', 'LineWidth', 1.75, 'DisplayName', sprintf('yl[%d]', i));                
                
                addBounds(time, sys.constraints.Y(i, :), configname);
                hold off;
                title(sprintf('Input %d', i)); xlabel('t'); ylabel(sprintf('y[%d]', i)); grid on; legend show;                
            end
            
            saveAndClose(output_dir, configname);
    end
end