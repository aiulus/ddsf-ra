% mode  -   Options: {
%                       'r':        vary det(R),
%                       'nt':       vary T_ini and N (prediction horizon) simultaneously,
%                       'constr':   multiply the pre-defined (by systemsDDSF.m)
%                                   input/output constraints with a scalar,
%                       'mixed':    vary all of the above simulteneously
%                     }
mode = 'nt';

% systype   -   OPTIONS:
%               linear: {'quadrotor', 'dampler', 'inverted_pendulum', 'dc_motor', 
%                        'cruise_control', 'acc', 'ballNbeam', 'double_pendulum'},
%               nonlinear: {'test_nonlinear', 'van_der_pol', 'nonlinear_pendulum'}
%
%            - Specifies the type of system for which the DDSF is being designed.
%           -  See algorithms\ddsf\systemsDDSF.m for details.
%systype = 'nonlinear_pendulum';
%systype = 'van_der_pol';
systype = 'quadrotor';

% Example definition of value ranges, extensive
vals = struct( ...
    'r', 10.^(-8:1:8), ... % value range for mode 'r'
    'NvsTini', [ ... % value range for mode 'nt'
    1 * ones(6, 1), (5:5:30)'; ...
    2 * ones(6, 1), (5:5:30)'; ...
    5 * ones(5, 1), (10:5:30)'; ...
    10 * ones(4, 1), (15:5:30)' ...
    ], ...
    'constraints', [1e+8, 0.1, 0.5, 1.5, 2, 10, 100], ... % value ranges for mode 'constr'
    'mixed', struct( ...
                    'nt', [repelem([1; 2], 4), repmat(5:5:20, 1, 2)'], ...
                    'constr', [1e+8, 0.1, 0.5, 1.5, 2], ...
                    'R', [1e-4, 1e-3,0.01, 0.1, 1, 10, 100, 1e+4, 1e+8] ...
                    ) ...
    );

% Example definition of value ranges, smaller
vals_small = struct( ...
    'r', 10.^(-2:1:2), ... % value range for mode 'r'
    'NvsTini', [ ... % value range for mode 'nt'
    2 * ones(6, 1), (5:5:30)'; ...
    5 * ones(5, 1), (10:5:30)'; .....
    ], ...
    'constraints', [0.1, 0.5,2], ... % value ranges for mode 'constr'
    'mixed', struct( ...
                    'nt', [repelem([2; 3], 4), repmat(5:5:20, 1, 2)'], ...
                    'constr', [0.1, 0.5,2], ...
                    'R', 10.^(-2:1:2) ...
                    ) ...
    );

% Number of simulation steps to be performed by (ddsfTunerFlex >) runDDSF.m
T_sim = 5;

% Whether the output CSV-file (containing simulation data) should be saved
% - configured to be true by default in ddsfTunerFlex.m
toggle_save = 1;

% Run (and save) the experiment with the given parameter configuration
[u, ul, y, yl, descriptions, filename] = ddsfTunerFlex(mode, vals_small, systype, T_sim, toggle_save);

% Extract the full path of the data files
filename_inputs = filename.u;
filename_outputs = filename.y;

% Construct (and save) plots from the data files
batchplot(filename_inputs);
batchplot(filename_outputs);
