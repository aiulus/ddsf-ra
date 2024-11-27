function debug_print(vars)
    disp('--- DEBUGGING INFORMATION ---');
    cellfun(@(name, val) fprintf('%s [%dx%d]:\n%g\n', name, size(val, 1), size(val, 2), val), ...
            fieldnames(vars), struct2cell(vars), 'UniformOutput', false);
    disp('-------------------------------');
end