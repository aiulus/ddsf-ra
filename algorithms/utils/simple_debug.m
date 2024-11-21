function simple_debug(M, name)
    fprintf("%s of order: ", name); 
    fprintf("[%d, %d]\n", size(M, 1), size(M, 2)); 
    disp(M), 
end

