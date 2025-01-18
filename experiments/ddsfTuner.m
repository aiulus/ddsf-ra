function [u, ul, descriptions] = ddsfTuner(mode, systype, T_sim, toggle_save)
    max_tries = 5;
    toggle_plot = false;
    
    if nargin < 4
        toggle_save = false;
    end
    
    vals = struct( ...
        'NvsTini', [ ...
        1 * ones(6, 1), (5:5:30)'; ...
        2 * ones(6, 1), (5:5:30)'; ...
        5 * ones(5, 1), (10:5:30)'; ...
        10 * ones(4, 1), (15:5:30)' ...
        ], ...
        'constraints', [1e+8, 0.1, 0.5, 1.5, 2, 10, 100], ...
        'mixed', struct( ...
        'nt', [repelem([1; 2], 4), repmat(5:5:20, 1, 2)'], ...
        'constr', [1e+8, 0.1, 0.5, 1.5, 2]) ...
        );
    
    switch lower(mode)
        case {'nvstini', 'nt'}
            values = vals.NvsTini;
            nruns = max(size(values));
        case {'constraints', 'constr'}
            values = vals.constraints;
            nruns = max(size(values));
        case 'mixed'
            values = vals.mixed;
            nruns = max(size(values.nt)) * max(size(values.constr));
        otherwise
            error("The provided tuner mode isn't supported.");
    end
    
    u = cell(1, nruns); ul = cell(1, nruns); descriptions = cell(1, nruns);
    
    output_dir = fullfile('..', 'outputs', 'plots', 'ddsf_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    switch mode
        case {'NvsTini', 'nt'}
            constraint_scaler = 1;
            tic;
            for i=1:nruns
                T_ini_i = values(i, 1);
                N_i = values(i, 2);
                d_i = sprintf('ddsfTuner-%s-N%d-Tini-%d-%s-T%d', mode, N_i, T_ini_i, systype, T_sim);
                fprintf('("------------------- Trying parameter conf. %d / %d -------------------\n', i, nruns);
                if i > 1
                    elapsed = toc;
                    timeEstimator(elapsed, i, nruns);
                end
                for k=1:max_tries
                    try
                        [~, ~, logs] = runDDSF(systype, T_sim, N_i, T_ini_i, constraint_scaler, toggle_plot);
                        u_i = logs.u; % m x T_sim
                        ul_i = logs.ul_t; % m x T_sim
                        u{i} = u_i;
                        ul{i} = ul_i;
                        descriptions{i} = d_i;
                        break;
                    catch ME
                        fprintf(['Attempt to run runDDSF.m (conf.: %d) failed at: %s\n ' ...
                            'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                    end
                end
            end
        case {'constraints', 'constr'}
            T_ini = -1; N = -1;
            tic;
            for i=1:nruns
                constr_i = values(i);
                d_i = sprintf('ddsfTuner-%s-scalingfactor%d-%s-T%d', mode, values(i), systype, T_sim);
                fprintf('("------------------- Trying parameter conf. %d / %d -------------------\n', i, nruns);
                if i > 1
                    elapsed = toc;
                    timeEstimator(elapsed, i, nruns);
                end
                for k=1:max_tries
                    try
                        [~, ~, logs] = runDDSF(systype, T_sim, N, T_ini, constr_i, toggle_plot);
                        u_i = logs.u; % m x T_sim
                        ul_i = logs.ul_t; % m x T_sim
                        u{i} = u_i;
                        ul{i} = ul_i;
                        descriptions{i} = d_i;
                    catch ME
                        fprintf(['Attempt to run runDDSF.m (conf.: %s) failed at: %s\n ' ...
                            'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                    end
                end
            end
        case 'mixed'            
            nt = values.nt;
            constr = values.constr;
            tic;
            for i=1:max(size(nt))
                T_ini_i = nt(i, 1);
                N_i = nt(i, 2);
                for j=1:max(size(constr))
                    constr_j = constr(j);
                    d_i = sprintf('ddsfTuner-%s-Tini%d-N%d-constr_scale%d-%s-T%d', mode, T_ini_i, N_i,constr_j, systype, T_sim);
                    t = (i-1)*max(size(nt))+j;
                    fprintf('("------------------- Trying parameter conf. %d / %d -------------------\n', t, nruns);
                    if t > 1
                        elapsed = toc;
                        timeEstimator(elapsed, t, nruns);
                    end
                    for k=1:max_tries
                        try
                            [~, ~, logs] = runDDSF(systype, T_sim, N_i, T_ini_i, constr_j, toggle_plot);
                            u_i = logs.u; % m x T_sim
                        ul_i = logs.ul_t; % m x T_sim
                        u{i} = u_i;
                        ul{i} = ul_i;
                        descriptions{i} = d_i;
                            break;
                        catch ME
                            fprintf(['Attempt to run runDDSF.m (conf.: %s) failed at: %s\n ' ...
                                'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                        end
                    end
                end
            end
    end
    if toggle_save
        prefix = sprintf('ddsfTuner-%s-%s-T%d', mode, systype, T_sim);
        % save2csv(u, ul, descriptions, prefix);
        csvFlexSave(prefix, u, ul, descriptions);
    end
end