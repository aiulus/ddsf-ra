function [u, y, descriptions] = deepcTunerFlex(mode, systype, T_sim, toggle_save, varargin)
    % Default parameters
    max_tries = 5;
    toggle_save = nargin > 3 && toggle_save;

    % Default configurations
    defaultParams.nt = [ ...
        1 * ones(6, 1), (5:5:30)'; ...
        2 * ones(6, 1), (5:5:30)'; ...
        5 * ones(5, 1), (10:5:30)'; ...
        10 * ones(4, 1), (15:5:30)' ...
    ];

    defaultParams.qr = table2array(combinations(logspace(-2, 4, 5), logspace(-2, 2, 5)));
    defaultParams.constraints = struct('umin', -1, 'umax', 1, 'ymin', -1, 'ymax', 1);

    % Parse additional input arguments 
    params = parseParams(defaultParams, varargin{:});

    % Generate all combinations based on the selected mode
    configurations = generateConfigurations(mode, params);

    % Total number of runs
    nruns = numel(configurations);

    % Preallocate outputs
    u = cell(1, nruns);
    y = cell(1, nruns);
    descriptions = cell(1, nruns);

    % Create output directory
    output_dir = fullfile('..', 'outputs', 'plots', 'deepc_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % Main execution loop
    tic;
    % parfor (i = 1:nruns, feature('numcores')) % Parallelize if possible
    for i = 1:nruns 
        fprintf('Trying parameter configuration %d / %d\n', i, nruns);
        try
            [u{i}, y{i}, descriptions{i}] = executeRun(systype, T_sim, configurations{i}, max_tries);
        catch ME
            fprintf('Configuration %d failed: %s\n', i, ME.message);
        end
    end

    % Save results if required
    if toggle_save
        prefix = sprintf('deepcTunerFlex-%s-%s-T%d', mode, systype, T_sim);
        saveResults(prefix, u, y, descriptions);
    end
end

%% Helper Functions
function params = parseParams(defaultParams, varargin)
    % Override default parameter ranges with user-defined values
    params = defaultParams;
    for i = 1:2:numel(varargin)
        key = varargin{i};
        if isfield(params, key)
            params.(key) = varargin{i+1};
        else
            error("Unknown parameter: '%s'", key);
        end
    end
end

function configurations = generateConfigurations(mode, params)
    switch mode
        case 'nt'
            configurations = arrayfun(@(i) struct('nt', params.nt(i, :), 'qr', [], 'constraints', []), ...
                1:size(params.nt, 1), 'UniformOutput', false);
        case 'qr'
            configurations = arrayfun(@(i) struct('nt', [], 'qr', params.qr(i, :), 'constraints', []), ...
                1:size(params.qr, 1), 'UniformOutput', false);
        case 'constraints'
            configurations = struct2cell(params.constraints)';
            configurations = cellfun(@(c) struct('nt', [], 'qr', [], 'constraints', c), configurations, ...
                'UniformOutput', false);
        case 'all'
            [ntIdx, qrIdx, cIdx] = ndgrid(1:size(params.nt, 1), 1:size(params.qr, 1), 1:numel(fieldnames(params.constraints)));
            configurations = arrayfun(@(n, q, c) struct( ...
                'nt', params.nt(n, :), ...
                'qr', params.qr(q, :), ...
                'constraints', params.constraints.(sprintf('constraint_%d', c))), ...
                ntIdx, qrIdx, cIdx, 'UniformOutput', false);
        case 'nt_qr'
            [ntIdx, qrIdx] = ndgrid(1:size(params.nt, 1), 1:size(params.qr, 1));
            configurations = arrayfun(@(n, q) struct( ...
                'nt', params.nt(n, :), ...
                'qr', params.qr(q, :), ...
                'constraints', []), ...
                ntIdx, qrIdx, 'UniformOutput', false);
        case 'nt_constraints'
            [ntIdx, cIdx] = ndgrid(1:size(params.nt, 1), 1:numel(fieldnames(params.constraints)));
            configurations = arrayfun(@(n, c) struct( ...
                'nt', params.nt(n, :), ...
                'qr', [], ...
                'constraints', params.constraints.(sprintf('constraint_%d', c))), ...
                ntIdx, cIdx, 'UniformOutput', false);
        case 'qr_constraints'
            [qrIdx, cIdx] = ndgrid(1:size(params.qr, 1), 1:numel(fieldnames(params.constraints)));
            configurations = arrayfun(@(q, c) struct( ...
                'nt', [], ...
                'qr', params.qr(q, :), ...
                'constraints', params.constraints.(sprintf('constraint_%d', c))), ...
                qrIdx, cIdx, 'UniformOutput', false);
        otherwise
            error("Unsupported mode '%s'.", mode);
    end
end

function [u_i, y_i, d_i] = executeRun(systype, T_sim, config, max_tries)
    % Run a single parameter configuration
    for attempt = 1:max_tries
        try
            % Generate description string
            d_i = sprintf('Config: nt=%s, qr=%s, constraints=%s', ...
                mat2str(config.nt), mat2str(config.qr), mat2str(config.constraints));

            % Run simulation
            logs = runParamDPC(systype, config.qr(1), config.qr(2), config.nt(1), config.nt(2), T_sim);
            u_i = logs.u_sim; % m x T_sim
            y_i = logs.y_sim; % m x T_sim
            return; % Successful execution
        catch
            if attempt == max_tries
                error('All attempts failed for configuration: %s', d_i);
            end
        end
    end
end

function saveResults(prefix, u, y, descriptions)
    % Prepare data for saving using csvFlexSave.
    nConfigs = numel(u); % Number of configurations
    saveInputs = cell(1, 3 * nConfigs); % To hold all u, y, and descriptions
    colIdx = 1; % Column index tracker
    
    % Flatten data for u and y vectors
    for i = 1:nConfigs
        % Add u vectors
        if ~isempty(u{i})
            for v = 1:size(u{i}, 1)
                saveInputs{colIdx} = u{i}(v, :);
                colIdx = colIdx + 1;
            end
        end
        
        % Add y vectors
        if ~isempty(y{i})
            for v = 1:size(y{i}, 1)
                saveInputs{colIdx} = y{i}(v, :);
                colIdx = colIdx + 1;
            end
        end
    end
    
    % Add descriptions as the last columns
    for i = 1:nConfigs
        saveInputs{colIdx} = descriptions{i};
        colIdx = colIdx + 1;
    end
    
    % Use csvFlexSave to save results
    csvFlexSave(prefix, saveInputs{:});
end
