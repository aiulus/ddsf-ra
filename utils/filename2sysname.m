function sysname = filename2sysname(filename)
    parts = split(filename, '-');

    if numel(parts) >= 6
        % Extract the substring between the fourth and fifth hyphens
        sysname = parts{5};
    else
        error('<filename2sysname.m>: The filename %s cannot be parsed.');
    end
end

