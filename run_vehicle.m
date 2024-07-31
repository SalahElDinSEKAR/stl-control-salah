clear all;
init_vehicle

init_sampling.num_corners= 16;
init_sampling.num_quasi_random = 84;
init_sampling.recompute = 0;
init_sampling.seed=5000;

falsif_sampling.num_corners = 16; 
falsif_sampling.num_quasi_random = 84;
falsif_sampling.seed = 10000;
falsif_sampling.recompute = 0;
falsif_sampling.local_max_obj_eval = 0; 
falsif_sampling.scaling = .8;

res= pb_vehicle.algo2(...
    'init_sampling', init_sampling, ...
    'falsif_sampling', falsif_sampling, ...
    'num_training',  2, ...
    'max_num_cex_traces', 20, ...
    'max_num_cex_samples', 20);
