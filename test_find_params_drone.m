% test the inverse param function for the drone example.
close all 
init_drone


init_sampling.num_quasi_random = 10;
init_sampling.num_corners= 0;
init_sampling.recompute = 0;
init_sampling.seed=5000;

B = pb_drone.get_mpc_traces(init_sampling)

%%
X = B.GetParam(1:12);

%%
[Pts, err] = pb_drone.get_params_from_X(X);

%%
Pfound = Pts(13:15,:);
Ptrue = B.GetParam(13:15); 