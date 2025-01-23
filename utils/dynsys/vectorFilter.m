function result = vectorFilter(testarr, x_e)
    % Initialize the result as a cell array of size 12x1
    result = cell(size(testarr));
    
    % Loop through each entry in x_eq
    for i = 1:length(x_e)
        % Check if the entry in x_eq is symbolic
        entry_class = string(class(x_e(i)));
        entry_value = string(value(x_e(i)));
        if entry_class == "sym" && startsWith(entry_value, 'z')
            % Keep the corresponding entry from x_eval
            result{i} = testarr(i);
        elseif startsWith(entry_value, 'k')
            syms k
            x_e(i) = subs(x_e(i), k, 1);
            result{i} = double(x_e(i));
        else            
            result{i} = double(x_e(i));
        end
    end
    result = cell2mat(result);
end