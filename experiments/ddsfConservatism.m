Ts = [1, 2, 5, 10];
Ns = [5, 10, 15, 20, 25, 30];

values = [
    1 * ones(6, 1), (5:5:30)';
    2 * ones(6, 1), (5:5:30)';
    5 * ones(5, 1), (10:5:30)';
    10 * ones(4, 1), (15:5:30)'
];

systype = "cruise_control";
T_sim = 10;

output_dir = fullfile('..', 'outputs', 'plots', 'conservatism');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

nruns = size(values, 1);
for i=1:nruns
    Tini = values(i, 1); N = values(i, 2);

    fprintf("------------------- Conservatism: %d / %d " + ...
        "-------------------", i, nruns);
    fprintf('Running DDSF with T_ini = %d and N = %d...\n', T_ini, N);

    try
        [time, logs] = runDDSF(systype, T_sim, N, Tini);

        % Generate unique file suffix for plots
        suffix = sprintf('Tini%d_N%d', T_ini, N);

        plotDDSF(time, logs, lookup);

        save_plots(output_dir, suffix);

    catch ME
        fprintf('Error encountered with T_ini = %d and N = %d:\n', T_ini, N);
        fprintf('%s\n', ME.message);
        continue; 
    end
end

