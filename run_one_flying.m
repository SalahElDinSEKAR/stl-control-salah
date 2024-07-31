clear all;
init_paths FlyingRobot_model
init_flying_problem

init_sampling1000.num_quasi_random = 744;
init_sampling1000.num_corners= 256;
init_sampling1000.seed=5000;

falsif_sampling.num_corners = 256; 
falsif_sampling.num_quasi_random = 744;
falsif_sampling.seed = 10000;
falsif_sampling.scaling = .8;

res= pb_flying.algo1('init_sampling', init_sampling1000, ...
    'falsif_sampling', falsif_sampling, ...
      'num_training', 1, ...
      'max_num_cex_traces', 1, ...
      'max_num_cex_samples', 10);




