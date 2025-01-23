function f_eval = createFunctionHandle(f_sym, vars)
    % Ensure `f_sym` is a cell array
    if ~iscell(f_sym)
        f_sym = {f_sym};
    end

    % Convert symbolic expressions to function handles
    f_eval = cellfun(@(f) matlabFunction(f, 'Vars', {vars}), f_sym, 'UniformOutput', false);

    % Debug: Check if f_eval is a valid cell array
    if ~iscell(f_eval)
        error('createFunctionHandle failed to produce a cell array of function handles.');
    end
end
