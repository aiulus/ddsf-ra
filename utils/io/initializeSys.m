function sys = initializeSys(mode, sysname)
    switch mode
        case {'ddsf', 'u-ddsf', 'y-ddsf'}
            if ismember(sysname, {'test_nonlinear', 'van_der_pol', 'nonlinear_pendulum'})
                sys = nonlinearSysInit(sysname);
            else
                sys = systemsDDSF(sysname, 0);
            end
        case 'deepc'
            sys = sysInit(sysname);
        otherwise
            error('Unsupported mode: %s', mode);
    end
end