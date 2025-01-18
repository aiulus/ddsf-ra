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
    
    d_csv = [];

    % Preprocess and validate each input
    for i = 1:nInputs
        col_i = varargin{i};
        col_i = col_i(:);    
        d_csv = [d_csv, col_i];
    end
    
    % Construct output directory
    output_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'outputs', 'data');
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Write data to CSV
    filename = fullfile(output_dir, sprintf('%s.csv', prefix));
    try
        writematrix(d_csv, filename, 'Delimiter', ',', 'WriteMode', 'overwrite');
    catch matrixME
        fprintf('WRITEMATRIFailed to write data to CSV: %s', matrixME.message);
        try
            writecell(d_csv, filename, 'Delimiter', ',', 'WriteMode', 'overwrite');
        catch cellME
             error('Failed to write data to CSV: %s', cellME.message);
        end
    end
end