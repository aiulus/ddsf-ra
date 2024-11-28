function debug_log(debug_toggle, iteration, log_interval, file, varargin)
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

    if debug_toggle && (mod(iteration, log_interval) == 0 || iteration == 1)
        fprintf('--- DEBUG: Iteration %d --- \n', iteration);
        
        if file
            fprintf(log_fid, '--- DEBUG: Iteration %d --- \n', iteration);
        end

        for i = 1:2:length(varargin)
            label = varargin{i};
            value = varargin{i + 1};
            
            % Handle different types of `value`
            if isa(value, 'sdpvar') % Symbolic variable (from YALMIP)
                value = value(value); % Evaluate the variable
            end
            
            if isnumeric(value) % Numeric data
                value_str = num2str(value(:)', '%.6g '); % Convert to string
            elseif islogical(value) % Logical data
                value_str = mat2str(value);
            elseif ischar(value) % Character data
                value_str = value;
            elseif iscell(value) % Cell arrays
                value_str = sprintf('Cell[%d]', numel(value));
            else
                value_str = sprintf('Unsupported type: %s', class(value));
            end

            fprintf('%s: [%s]\n', label, value_str);
            if file
                fprintf(log_fid, '%s: [%s]\n', label, value_str);
            end
        end

        %for i = 1:2:length(varargin)
         %   label = varargin{i};
          %  value = varargin{i + 1};
           % fprintf('%s: [%s]\n', label, join(string(value(:)), ','));
            %if file
             %   fprintf(log_fid, '%s: [%s]\n', label, join(string(value(:)), ','));
            %end
        %end

        fprintf('--------------------------\n');
        if file
            fprintf(log_fid, '--------------------------\n');
        end
    end
end


