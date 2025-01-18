%% Retrieves individual sequences from a CSV file
% Inputs:
%   filename: The path to the CSV file.
%
% Outputs:
%   data: A cell array where each cell contains one sequence.
%   sequenceNames: A cell array of sequence names for reference.
%
% This function reads a CSV file saved with `csvFlexSave` and extracts individual
% sequences as numeric arrays.

function [data, sequenceNames] = csvFlexRead(filename)
    % Check if file exists
    if ~isfile(filename)
        error('The file %s does not exist.', filename);
    end

    try
        fullPath = which(filename);
        if isempty(fullPath)
            % If `which` fails, try resolving relative path
            fullPath = fullfile(pwd, filename);
        end
        if ~isfile(fullPath)
            error('File not found. Ensure the file path is correct: %s', fullPath);
        end
    catch
        error('Error resolving full path for the file. Verify the input filename.');
    end
    
    % Read the CSV file into a table
    try
        dataTable = readtable(fullPath);
    catch err
        error('Error reading the CSV file: %s', err.message);
    end
    
    % Extract sequence names (column headers)
    sequenceNames = dataTable.Properties.VariableNames;
    
    % Convert each column into a numeric array
    nSequences = numel(sequenceNames);
    data = cell(1, nSequences);
    for i = 1:nSequences
        columnData = dataTable.(sequenceNames{i});
        data{i} = columnData(~isnan(columnData)); % Remove NaN padding
    end
    
    fprintf('Successfully retrieved %d sequences from %s\n', nSequences, filename);
end
