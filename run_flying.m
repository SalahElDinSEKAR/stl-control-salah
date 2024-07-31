clear all;
init_paths FlyingRobot_model
%%
pb_flying =  FlyingRobot_imitation_pb(30); % number of cells in each dim

pb_flying =  FlyingRobot_imitation_pb(20);

%%
init_sampling1000.num_corners= 256;
init_sampling1000.num_quasi_random = 744;
init_sampling1000.seed=5000;

init_sampling100.num_corners= 16;
init_sampling100.num_quasi_random = 84;
init_sampling100.seed=5000;

%%
falsif_sampling.num_corners = 256; 
falsif_sampling.num_quasi_random = 744;
falsif_sampling.seed = 10000;
falsif_sampling.scaling = .8;
falsif_sampling.local_max_obj_eval = 100;
falsif_sampling.max_obj_eval = 1000;
falsif_sampling.freq_update =  10; 
falsif_sampling.display = 'on';
%%

options = struct( ...
    'init_sampling', init_sampling1000, ...
    'falsif_sampling', falsif_sampling,...
    'num_random_traces',50,...
    'max_num_cex_traces',50,...
    'max_num_cex_samples',100,...
    'num_training', 5);

%%
load_last =1; 
if load_last
    load('last_results_algo2.mat');
    options.resume_results = {res};
end

%%
all_results = pb_flying.algo2(options);

%%
options.resume_results = all_results;  % run the line above again for another round with same options
options.num_training = 10;  % run the line above again for another round with same options


