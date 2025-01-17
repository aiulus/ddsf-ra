function [u, ul, descriptions] = ddsfTuner(mode, systype, T_sim)
    max_tries = 5;
    toggle_plot = false;

    u = {}; ul = {}; descriptions = {};
    
    vals = struct( ...
            'NvsTini', [ ...
                1 * ones(6, 1), (5:5:30)'; ...
                2 * ones(6, 1), (5:5:30)'; ...
                5 * ones(5, 1), (10:5:30)'; ...
                10 * ones(4, 1), (15:5:30)' ...
            ], ...
            'constraints', [1e+8, 0.1, 0.5, 1.5, 2, 10, 100] ...
    );

    switch lower(mode)
        case 'nvstini'
            values = vals.NvsTini;
        case 'constraints'
            values = vals.constraints;
        otherwise
            error("The provided tuner mode isn't supported.");
    end

    nruns = max(size(values));

    output_dir = fullfile('..', 'outputs', 'plots', 'ddsf_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    switch mode
        case 'NvsTini'
            constraint_scaler = 1;
            for i=1:nruns
               T_ini_i = values(i, 1);
               N_i = values(i, 2);
               d_i = sprintf('%s-N%d-Tini-%d-%s-T%d', mode, N_i, T_ini_i, systype, T_sim);
               for k=1:max_tries
                    try
                        [~, ~, logs] = runDDSF(systype, T_sim, N_i, T_ini_i, constraint_scaler, toggle_plot);
                        u_i = logs.u; % m x T_sim
                        ul_i = logs.ul_t; % m x T_sim                
                        u{end} = u_i;
                        ul{end} = ul_i;
                        descriptions{end} = d_i;
                        k = max_tries;
                    catch ME
                        fprintf('Attempt to run DDSF failed: %s ME = %s\n. Trying again...', d_i, ME.message);
                    end
               end
            end
        case 'constraints'
            T_ini = -1; N = -1;
            for i=1:nruns
                d_i = sprintf('%s-scalingfactor%d-%s-T%d', mode, values(i), systype, T_sim);
                for k=1:max_tries
                    try
                        [~, ~, logs] = runDDSF(systype, T_sim, N, T_ini, values(i), toggle_plot);
                        u_i = logs.u; % m x T_sim
                        ul_i = logs.ul_t; % m x T_sim                
                        u{end} = u_i;
                        ul{end} = ul_i;
                        descriptions{end} = d_i;
                    catch ME
                        fprintf('Attempt to run DDSF failed: %s ME = %s\n. Trying again...', d_i, ME.message);
                    end
                end
            end
    end
end