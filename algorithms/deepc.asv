%% DeePC algorithm

%% Step 0: Define hyperparameters
T_ini = 5; % Length of initial trajectory
N = 20; % Prediction horizon

%% Step 1: Data collection
data_length = 50; % Length of the data sequence for training
% Generate [data_length] data points from an arbitrary system
[u_d, y_d] = datagen_cruise_control(data_length);

%% Step 2: Generate the Hankel matrices
[Up, Yp, Uf, Yf] = deepc_hankel(u_d, y_d, T_ini, N);


