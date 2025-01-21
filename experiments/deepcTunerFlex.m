function [u, y, descriptions, filename] = deepcTunerFlex(mode, vals, systype, T_sim, toggle_save)
    % Main entry point for the DeepC Tuner.
    %
    % INPUTS:
    %   mode        - String specifying the tuning mode ('NvsTini', 'QvsR', 'mixed').
    %   vals        - Struct containing pre-defined values for parameters.
    %   systype     - String specifying the system type.
    %   T_sim       - Simulation time.
    %   toggle_save - (Optional) Boolean to toggle saving results.
    %
    % OUTPUTS:
    %   u           - Cell array of control inputs for each run.
    %   y           - Cell array of system outputs for each run.
    %   descriptions - Cell array of descriptions for parameter configurations.

    % Default value for toggle_save
    if nargin < 5, toggle_save = true; end    

    % Validate inputs
    validateInputs(mode, vals);

    % Determine configurations based on mode
    [values, nruns, param_struct] = getModeConfig(mode, vals);

    % Initialize outputs
    u = cell(1, nruns);
    y = cell(1, nruns);
    descriptions = cell(1, nruns);

    % Output directory setup
    output_dir = fullfile('..', 'outputs', 'plots', 'deepc_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % Main loop for parameter tuning
    tic;
    for i = 1:nruns
        fprintf('------------------- Trying parameter conf. %d / %d -------------------\n', i, nruns);
        if i > 1
            elapsed = toc;
            timeEstimator(elapsed, i, nruns);
        end

        % Extract parameters for this run
        params = extractParameters(mode, i, values, param_struct);

        % Description generation
        d_i = createDescription(mode, systype, T_sim, params);

        % Execute and handle retries
        [u{i}, y{i}] = executeWithRetries(systype, T_sim, params, d_i);

        % Store description
        descriptions{i} = d_i;
    end

    % Save results if toggled
    if toggle_save
        prefix = sprintf('deepcTuner-%s-%s-T%d', mode, systype, T_sim);
        filename = csvFlexSave(prefix, u, y, descriptions);
    end

    % Convert outputs to matrices
    u = cell2mat(u(:));
    y = cell2mat(y(:));
end

%% Helper Functions

function validateInputs(mode, vals)
    % Validates the user inputs for mode and vals.

    % Check mode validity
    valid_modes = {'NvsTini', 'QvsR', 'mixed', 'nt', 'qr'};
    if ~ismember(mode, valid_modes)
        error('Invalid mode "%s". Supported modes are: %s.', mode, strjoin(valid_modes, ', '));
    end

    % Check vals structure
    required_fields = {'NvsTini', 'QvsR', 'mixed'};
    for field = required_fields
        if ~isfield(vals, field{1})
            error('The vals structure is missing the required field "%s".', field{1});
        end
    end
end

function [values, nruns, param_struct] = getModeConfig(mode, vals)
    % Maps the mode to the appropriate parameter values and configurations.

    switch mode
        case {'NvsTini', 'nt'}
            values = vals.NvsTini;
            nruns = size(values, 1);
            param_struct = struct('Q', -1, 'R', -1);

        case {'QvsR', 'qr'}
            values = vals.QvsR;
            nruns = size(values, 1);
            param_struct = struct('T_ini', -1, 'N', -1);

        case 'mixed'
            values = vals.mixed;
            nruns = size(values.qr, 1) * size(values.nt, 1);
            param_struct = struct('qr', values.qr, 'nt', values.nt);

        otherwise
            error('Unsupported mode "%s".', mode);
    end
end

function params = extractParameters(mode, index, values, param_struct)
    % Extracts parameters for the current configuration based on the mode and index.

    switch mode
        case {'NvsTini', 'nt'}
            params = struct('T_ini', values(index, 1), 'N', values(index, 2), 'Q', param_struct.Q, 'R', param_struct.R);

        case {'QvsR', 'qr'}
            params = struct('Q', values(index, 1), 'R', values(index, 2), 'T_ini', param_struct.T_ini, 'N', param_struct.N);

        case 'mixed'
            [qr_idx, nt_idx] = ind2sub([size(param_struct.qr, 1), size(param_struct.nt, 1)], index);
            params = struct('T_ini', param_struct.nt(nt_idx, 1), 'N', param_struct.nt(nt_idx, 2), ...
                            'Q', param_struct.qr(qr_idx, 1), 'R', param_struct.qr(qr_idx, 2));
    end
end

function d_i = createDescription(mode, systype, T_sim, params)
    % Generates a description string for the current configuration.

    if strcmp(mode, 'mixed')
        d_i = sprintf('deepcTuner-%s-N%d-Tini%d-Q%.2f-R%.2f-%s-T%d', ...
                      mode, params.N, params.T_ini, params.Q, params.R, systype, T_sim);
    elseif ismember(mode, {'NvsTini', 'nt'})
        d_i = sprintf('deepcTuner-%s-N%d-Tini%d-%s-T%d', mode, params.N, params.T_ini, systype, T_sim);
    else
        d_i = sprintf('deepcTuner-%s-Q%.2f-R%.2f-%s-T%d', mode, params.Q, params.R, systype, T_sim);
    end
end

function [u_i, y_i] = executeWithRetries(systype, T_sim, params, d_i)
    % Executes the simulation with retries in case of errors.

    max_tries = 5;
    for k = 1:max_tries
        try
            logs = runParamDPC(systype, params.Q, params.R, params.T_ini, params.N, T_sim);
            u_i = logs.u_sim;
            y_i = logs.y_sim;
            return;
        catch ME
            fprintf(['Attempt to run runParamDPC.m (conf.: %s) failed at: %s\n ' ...
                     'Message: = %s\n. Trying again...\n'], d_i, ME.stack(1).name, ME.message);
        end
    end
    error('Failed after %d attempts for configuration "%s".', max_tries, d_i);
end
