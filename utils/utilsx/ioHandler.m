function [optimizer_type, constr_type] = ioHandler()
    disp('Please select constraint mode.');
    constr_type = '';

    while ~ismember(lower(constr_type), {'f', 's', 'e'})
        constr_type = input(['Type <f> to include input/output constraints' ...
                            ' <s> to just encode system dynamics and' ...
                            ' <e> for an empty constraint set.'], 's');       
    end

    disp('Please select solver type: ');
    optimizer_type = '';

    while ~ismember(lower(optimizer_type), {'q', 'f', 'o'})
        optimizer_type = input(['Enter "q" for quadprog, ' ...
            '"f" for fmincon or "o" for osqp: '], 's');
    end
end