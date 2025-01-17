function [u, y, descriptions] = deepcTuner(mode, systype, T_sim, toggle_save)
    max_tries = 5;

    if nargin < 4
        toggle_save = false;
    end
    
    u = {}; y = {}; descriptions = {};
    
    Qs = logspace(-2, 4, 7);
    Rs = logspace(-2, 2, 5);
    
    vals = struct( ...
        'NvsTini', [ ...
            1 * ones(6, 1), (5:5:30)'; ...
            2 * ones(6, 1), (5:5:30)'; ...
            5 * ones(5, 1), (10:5:30)'; ...
            10 * ones(4, 1), (15:5:30)' ...
        ], ...
        'QvsR', table2array(combinations(Qs, Rs)), ...
        'mixed', struct( ...
            'nt', [ ...
                2 * ones(6, 1), (5:5:30)'; ...
                5 * ones(5, 1), (10:5:30)' ...
            ], ...
            'qr', table2array(combinations(logspace(-2, 1, 4), logspace(-2, 1, 4))) ...
        ) ...
    );


    switch mode
        case {'NvsTini', 'nt'}
            values = vals.NvsTini;
            nruns = max(size(values));
        case {'QvsR', 'qr'}
            values = vals.constraints;
            nruns = max(size(values));
        case 'mixed'
            values = vals.mixed;
            nruns = max(size(values.nt)) * max(size(values.qr));
        otherwise
            error("The provided tuner mode isn't supported.");
    end

    output_dir = fullfile('..', 'outputs', 'plots', 'deepc_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    switch mode
        case {'NvsTini', 'nt'}
            Q = -1; R = -1;
            for i=1:nruns
               T_ini_i = values(i, 1);
               N_i = values(i, 2);
               d_i = sprintf('deepcTuner-%s-N%d-Tini-%d-%s-T%d', mode, N_i, T_ini_i, systype, T_sim);
               for k=1:max_tries
                    try
                        logs = runParamDPC(systype, Q, R, T_ini_i, N_i, T_sim, toggle_save);
                        u_i = logs.u_sim; % m x T_sim
                        y_i = logs.y_sim; % m x T_sim                
                        u{end} = u_i;
                        y{end} = y_i;
                        descriptions{end} = d_i;
                        break;
                    catch ME
                        fprintf(['Attempt to run runParamDPC.m (conf.: %s) failed at: %s\n ' ...
                            'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                    end
               end
            end
        case {'QvsR', 'qr'}
            T_ini = -1; N = -1;
            for i=1:nruns
                d_i = sprintf('%deepcTuner-s-Q%d-R%d-%s-T%d', mode, values(i), systype, T_sim);
                Q_i = values(i, 1);
                R_i = values(i, 2);
                for k=1:max_tries
                    try
                        logs = runParamDPC(systype, Q_i, R_i, T_ini, N, T_sim, toggle_save);
                        u_i = logs.u_sim; % m x T_sim
                        y_i = logs.y_sim; % m x T_sim                
                        u{end} = u_i;
                        y{end} = y_i;
                        descriptions{end} = d_i;
                        break;
                    catch ME
                         fprintf(['Attempt to run DDSF (conf.: %s) failed at: %s\n ' ...
                            'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                    end
                end
            end
        case 'mixed'
            qr = values.qr;
            nt = values.nt;
            for i=1:nruns
               Q_i = qr(i, 1); R_i = qr(i, 2);
               T_ini_i = nt(i, 1); N_i = nt(i, 2);               
               d_i = sprintf('deepcTuner-%s-N%d-Tini-%d-Q%d-R%d-%s-T%d', mode, N_i, T_ini_i, Q_i, R_i, systype, T_sim);
               for k=1:max_tries
                    try
                        logs = runParamDPC(systype, Q_i, R_i, T_ini_i, N_i, T_sim, toggle_save);
                        u_i = logs.u_sim; % m x T_sim
                        y_i = logs.y_sim; % m x T_sim                
                        u{end} = u_i;
                        y{end} = y_i;
                        descriptions{end} = d_i;
                        break;
                    catch ME
                        fprintf(['Attempt to run runParamDPC.m (conf.: %s) failed at: %s\n ' ...
                            'Message: = %s\n. Trying again...'], d_i, ME.stack(1).name, ME.message);
                    end
               end
            end
    end
end
