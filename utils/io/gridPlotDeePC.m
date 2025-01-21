function gridPlotDeePC(configname, sys, u_hist, y_hist)
    output_dir = prepareOutputDir();
    figure; tiledlayout(size(y_hist, 1), 1);

    for i = 1:size(y_hist, 1)
        nexttile;
        plot(0:size(y_hist, 2) - 1, y_hist(i, :), 'b', 'LineWidth', 1.25, 'DisplayName', sprintf('y[%d]', i));
        addTargetLine(sys.target, size(y_hist, 2));
        title(sprintf('Output %d', i)); xlabel('t'); ylabel(sprintf('y[%d]', i)); grid on; legend show;
    end
    saveAndClose(output_dir, configname);
end