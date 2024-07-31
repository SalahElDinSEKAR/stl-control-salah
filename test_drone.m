clear all;
init_drone

init_sampling.num_quasi_random = 84;
init_sampling.num_corners= 16;
init_sampling.seed=5000;

B = pb_drone.test_nominal();
