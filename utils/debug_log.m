function debug_log(iteration, log_interval, debug_mode, file, varargin)
    % DEBUG_LOG Systematically logs data points during execution.
    % 
    % INPUTS:
    % iteration     - Current iteration of the loop.
    % log_interval  - Frequency of logging (e.g., every 10 iterations).
    % debug_mode    - Flag to enable/disable debug logging.
    % varargin      - Pairs of {'label', value} to log data points.
    %
    % Example:
    % debug_log(t, 10, true, {'u_ini', u_ini, 'y_ini', y_ini});
    
    if file
        log_file = 'debug_log.txt';
        log_fid = fopen(log_file, 'a');
    end

    if debug_mode && (mod(iteration, log_interval) == 0 || iteration == 1)
        fprintf('--- DEBUG: Iteration %d --- \n', iteration);
        
        if file
            fprintf(log_fid, '--- DEBUG: Iteration %d --- \n', iteration);
        end

        for i = 1:2:length(varargin)
            label = varargin{i};
            value = varargin{i + 1};
            fprintf('%s: [%s]\n', label, join(string(value(:)), ','));
            if file
                fprintf(log_fid, '%s: [%s]\n', label, join(string(value(:)), ','));
            end
        end
        fprintf('--------------------------\n');
        if file
            fprintf(log_fid, '--------------------------\n');
        end
    end
end

