%% Initialize flying robot problem 
% don't forget to change breach path in init_paths.m

init_flying_problem;

%% Compute or load traces with MPC from nominal initial state set
% if parameters matches in data folder, will just load the file
Bmpc_traces = pb_flying.get_mpc_data('num_corners',256, 'num_quasi_random', 744, 'seed', 5000);

%% Filter data 
% transform traces into samples, then filter them through the grid. See
% grid resolution in FlyingRobot_imitation_pb.m
[Bsamples, Bgrid]  = pb_flying.get_grid_data(Bmpc_traces);

%% Extract raw data
[x_in , u_out] = pb_flying.prepare_training_data(Bsamples);


