function sysname = filename2sysname(filename)
    parts = split(filename, '-');

    if numel(parts) >= 4
        % Extract the substring between the fourth and fifth hyphens
        sysname = parts{3};
    else
        error('<filename2sysname.m>: The filename %s cannot be parsed.');
    end
end



