function [u, ul, descriptions] = ddsfTunerFlex(mode, systype, T_sim, toggle_save, varargin)
    % Default settings
    max_tries = 5;
    toggle_save = nargin > 3 && toggle_save;

    % Default configurations
    defaultParams.nt = [ ...
        1 * ones(6, 1), (5:5:30)'; ...
        2 * ones(6, 1), (5:5:30)'; ...
        5 * ones(5, 1), (10:5:30)'; ...
        10 * ones(4, 1), (15:5:30)' ...
    ];
    defaultParams.constraints = [1e+8, 0.1, 0.5, 1.5, 2, 10, 100];
    defaultParams.r = [0.01, 0.1, 1, 10, 100, 1e+8];
    defaultParams.nt_constr = struct( ...
        'nt', [repelem([1; 2], 4), repmat(5:5:20, 1, 2)'], ...
        'constr', [1e+8, 0.1, 0.5, 1.5, 2] ...
    );

    % Parse user-defined parameters
    params = parseParams(defaultParams, varargin{:});

    % Generate configurations based on mode
    configurations = generateConfigurations(mode, params);

    % Total number of runs
    nruns = numel(configurations);

    % Preallocate outputs
    u = cell(1, nruns);
    ul = cell(1, nruns);
    descriptions = cell(1, nruns);

    % Create output directory
    output_dir = fullfile('..', 'outputs', 'plots', 'ddsf_tuner');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % Execution loop
    tic;
    for i = 1:nruns
        fprintf('Trying parameter configuration %d / %d\n', i, nruns);
        try
            % Execute configuration
            [u{i}, ul{i}, descriptions{i}] = executeRun(systype, T_sim, configurations{i}, max_tries);
        catch ME
            fprintf('Configuration %d failed: %s\n', i, ME.message);
        end
    end

    % Save results if requested
    if toggle_save
        prefix = sprintf('ddsfTunerFlex-%s-%s-T%d', mode, systype, T_sim);
        saveResults(prefix, u, ul, descriptions);
    end
end

%% Helper Functions
function params = parseParams(defaultParams, varargin)
    % Parse user-provided parameter overrides
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
    % Generate configurations based on the selected mode
    switch lower(mode)
        case {'nvstini', 'nt'}
            configurations = arrayfun(@(i) struct('nt', params.nt(i, :), 'constraints', [], 'r', []), ...
                1:size(params.nt, 1), 'UniformOutput', false);
        case {'constraints', 'constr'}
            configurations = arrayfun(@(i) struct('nt', [], 'constraints', params.constraints(i), 'r', []), ...
                1:numel(params.constraints), 'UniformOutput', false);
        case 'r'
            configurations = arrayfun(@(i) struct('nt', [], 'constraints', [], 'r', params.r(i)), ...
                1:numel(params.r), 'UniformOutput', false);
        case 'nt_r'
            [ntIdx, rIdx] = ndgrid(1:size(params.nt, 1), 1:numel(params.r));
            configurations = arrayfun(@(n, r) struct( ...
                'nt', params.nt(n, :), 'constraints', [], 'r', params.r(r)), ...
                ntIdx, rIdx, 'UniformOutput', false);
        case 'constr_r'
            [constrIdx, rIdx] = ndgrid(1:numel(params.constraints), 1:numel(params.r));
            configurations = arrayfun(@(c, r) struct( ...
                'nt', [], 'constraints', params.constraints(c), 'r', params.r(r)), ...
                constrIdx, rIdx, 'UniformOutput', false);
        case 'nt_constr'
            [ntIdx, constrIdx] = ndgrid(1:size(params.nt_constr.nt, 1), 1:numel(params.nt_constr.constr));
            configurations = arrayfun(@(n, c) struct( ...
                'nt', params.nt_constr.nt(n, :), ...
                'constraints', params.nt_constr.constr(c), 'r', []), ...
                ntIdx, constrIdx, 'UniformOutput', false);
        case 'all'
            [ntIdx, constrIdx, rIdx] = ndgrid(1:size(params.nt, 1), 1:numel(params.constraints), 1:numel(params.r));
            configurations = arrayfun(@(n, c, r) struct( ...
                'nt', params.nt(n, :), 'constraints', params.constraints(c), 'r', params.r(r)), ...
                ntIdx, constrIdx, rIdx, 'UniformOutput', false);
        otherwise
            error("Unsupported mode '%s'.", mode);
    end
end

function [u_i, ul_i, d_i] = executeRun(systype, T_sim, config, max_tries)
    % Execute a single configuration with retries
    for attempt = 1:max_tries
        try
            % Generate description
            d_i = sprintf('Config: nt=%s, constraints=%s, r=%s', ...
                mat2str(config.nt), mat2str(config.constraints), mat2str(config.r));

            % Run DDSF simulation
            [~, ~, logs] = runDDSF(systype, T_sim, ...
                config.nt(2), config.nt(1), config.constraints, config.r);
            u_i = logs.u;   % m x T_sim
            ul_i = logs.ul_t; % m x T_sim
            return; % Success
        catch
            if attempt == max_tries
                error('All attempts failed for configuration: %s', d_i);
            end
        end
    end
end

function saveResults(prefix, u, ul, descriptions)
    % Save results to CSV
    csvFlexSave(prefix, u, ul, descriptions);
end
