%% DeePC algorithm

%% Step 1: Data collection
data_length = 50; % Length of the data sequence for training
prbs_input = 25000 * (idinput(data_length, 'prbs') + 1); % PRBS input, scaled

% Initialize arrays for storing input-output data
u_d = zeros(1, data_length);
y_d = zeros(1, data_length);

% Initialize state (here: initial velocity)
x_data = 0; % TODO: shouldn't be hard-coded
            % TODO: built-in initialization procedure for each dynamical system


