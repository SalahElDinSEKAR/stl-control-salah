init_flying;
load data/Res_Flying_Success15-03.mat

init_sampling100.num_corners= 16;
init_sampling100.num_quasi_random = 84;
init_sampling100.scaling = .8;
init_sampling100.seed=5000;

init_sampling1000.num_corners= 256;
init_sampling1000.num_quasi_random = 744;
init_sampling1000.scaling = .8;
init_sampling1000.seed=1;

% MPC evaluation
[Bmpc, R] = pb_flying.get_mpc_traces(init_sampling1000);

%%
R = BreachRequirement('phi_over');
og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
R.AddPostProcess(og);
synth_pb = ParamSynthProblem(Bmpc, R, {'p_over'},  [5 15]);
p_mpc = synth_pb.solve();

%%

R = BreachRequirement('phi_stab');
og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
R.AddPostProcess(og);
synth_pb = ParamSynthProblem(Bmpc, R, {'p_'},  [10 15]);
p_mpc = synth_pb.solve();

%%
for inn = 1000:numel(all_results)
    nn = all_results{inn}.nn;
    [Bcex, Bnn] = pb_flying.get_nn_cex_traces( nn, init_sampling1000);
    Rover = BreachRequirement('phi_over');
    synth_pb = ParamSynthProblem(Bnn, Rover, {'p_over'},  [10 100]);
    pnn(inn)= synth_pb.solve();
end

