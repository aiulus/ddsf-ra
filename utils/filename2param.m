function param = filename2param(filename, mode)
    [~, filename, ext] = fileparts(filename);
    filename = strcat(filename, ext);

    switch lower(mode)
        case 'sys'
            pattern = '-systype-(?<name>[^-]+)-';
        case 'alg'
            if startsWith(filename, 'U-ddsf')
                param = 'ddsf';
                return;
            elseif startsWith(filename, 'Y-ddsf')
                param = 'ddsf';
                return;
            elseif startsWith(filename, 'ddsf')
                param = 'ddsf';
                return;
            elseif startsWith(filename, 'deepc')
                param = 'deepc';
                return;
            else
                warning('No valid algorithm type found.');
                param = '';
                return;
            end
        case 'tini'
            pattern = 'Tini(?<value>\d+)-';
        case 'n'
            pattern = '-N(?<value>\d+)-';
        case 'constr'
            % pattern = '-constr_scale(?<value>\d+)-';
            pattern = '-constr_scale(?<value>[\d\.eE+-]+)-';
        case 'r'
            pattern = '-R(?<value>\d+)-';
        case 't'
            pattern = '-T(?<value>\d+)-';
        otherwise
            error('Unsupported mode: %s. Valid modes are {alg, Tini, N, constr, R, T}.', mode);
    end
    
    %match = regexp(filename, pattern, 'names');
    
    if exist('pattern', 'var')
        match = regexp(filename, pattern, 'names');
        if isempty(match) || ~isfield(match, 'value')
            warning('No match found for mode "%s" in the provided string.', mode);
            param = []; 
        else
            param = str2double(match.value);
        end
    end
end