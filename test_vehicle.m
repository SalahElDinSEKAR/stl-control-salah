clear all;
init_vehicle

init_sampling.num_quasi_random = 84;
init_sampling.num_corners= 16;
init_sampling.seed=5000;

B0 = pb_vehicle.create_nominal();

B0.SampleDomain(10);
B0.Sim();
%%
pb_vehicle.plot_traces(B0)

                                                                                                                                                                                                                                                                                                            
