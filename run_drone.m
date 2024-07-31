clear all;
init_drone

init_sampling.num_corners= 16;
init_sampling.num_quasi_random = 84;
init_sampling.recompute = 0;
init_sampling.seed=5000;

falsif_sampling.num_corners = 16; 
falsif_sampling.num_quasi_random = 84;
falsif_sampling.seed = 10000;
falsif_sampling.recompute = 0;
falsif_sampling.scaling = .8;

res= pb_drone.algo2(...
    'init_sampling', init_sampling, ...
    'falsif_sampling', falsif_sampling, ...
    'num_training',  50, ...
    'max_num_cex_traces', 10, ...
    'max_num_cex_samples', 1);
