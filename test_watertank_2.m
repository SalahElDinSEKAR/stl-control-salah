clear all;
init_watertank;

init_sampling.num_quasi_random = 84;
init_sampling.num_corners= 16;
init_sampling.seed=5000;

Bpid = pb_watertank.create_nominal();
Bpid.SampleDomain(10);
% Bpid.QuasiRandomSample(100);
Bpid.Sim();
%%
pb_watertank.plot_traces(Bpid)
%%
G = BreachSignalsPlot(Bpid,'H');
G.AddSignals('ref',1);
STL_ReadFile req_watertank.stl
R = BreachRequirement('phi')
R.Eval(Bpid);
BreachSamplesPlot(R)