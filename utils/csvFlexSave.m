function csvFlexSave(prefix, varargin)
    %% Saves multiple sequences to a structured CSV file
    % Inputs:
    %   prefix: A string used as the base filename.
    %   varargin: A variable number of data inputs (can be numeric vectors or cells).
    
    if nargin < 2
        error('At least one data vector and a prefix must be provided.');
    end
    
    % Validate prefix
    if isempty(prefix) || ~ischar(prefix)
        error('The prefix must be a non-empty string.');
    end
    
    % Initialize structured data storage
    nInputs = numel(varargin);
    columnData = cell(1, nInputs);
    columnTypes = cell(1, nInputs);
    maxLen = 0;
    
    % Preprocess and validate each input
    for i = 1:nInputs
        data = varargin{i};
    
        % Handle cell inputs
        if iscell(data)
            try
                if all(cellfun(@ischar, data)) % Cell of strings
                    data = string(data); % Convert to string array
                else
                    data = cell2mat(data); % Convert to numeric array
                end
            catch
                error('Input %d could not be converted to a valid format (string or numeric).', i);
            end
        end
    
        % Handle string inputs
        if isstring(data) || ischar(data)
            columnData{i} = string(data(:)); % Convert to a column string array
            columnTypes{i} = 'string';
        elseif isnumeric(data)
            columnData{i} = data(:); % Force numeric data to a column vector
            columnTypes{i} = 'numeric';
        else
            error('Input %d is not a valid type (numeric or string).', i);
        end
    
        % Update maximum length
        maxLen = max(maxLen, numel(columnData{i}));
    end
    
    % Pad shorter vectors with NaNs
    paddedData = NaN(maxLen, nInputs);
    for i = 1:nInputs
        len = numel(columnData{i});
        paddedData(1:len, i) = columnData{i};
    end
    
    % Metadata for identification
    metadata = arrayfun(@(x) sprintf('Sequence_%d', x), 1:nInputs, 'UniformOutput', false);
    
    % Construct output directory
    output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'outputs', 'data');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Write data to CSV
    filename = fullfile(output_dir, sprintf('%s.csv', prefix));
    try
        % Combine metadata and data
        dataTable = array2table(paddedData, 'VariableNames', metadata);
        writetable(dataTable, filename, 'WriteVariableNames', true);
        fprintf('Data saved successfully to %s\n', filename);
    catch err
        error('Failed to write data to CSV: %s', err.message);
    end
end
