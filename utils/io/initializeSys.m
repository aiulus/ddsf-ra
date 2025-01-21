function sys = initializeSys(mode, sysname)
    switch mode
        case {'ddsf', 'u-ddsf', 'y-ddsf'}
            sys = systemsDDSF(sysname, 0);
        case 'deepc'
            sys = sysInit(sysname);
        otherwise
            error('Unsupported mode: %s', mode);
    end
end