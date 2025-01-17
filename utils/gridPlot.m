function gridPlot(mode, filepath, T_sim, m, p)
    data = readtable(filepath, 'ReadVariableNames', false);

    out1 = data{:, 1:T_sim}; % Input u                 
    out2 = data{:, T_sim+1:2*T_sim}; % (DDSF) u_l // (DeePC) y        
    configs = data{:, end}; 

    switch mode
        case 'ddsf'
            uX = out1; ulX = out2;
            for i=1:T_sim
                u_i = u
            end
        case 'deepc'
    end
end

