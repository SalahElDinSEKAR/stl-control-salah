init_watertank;
Bpid = pb_watertank.create_nominal();

Bpid.QuasiRandomSample(100);
Bpid.Sim();

%%

G = BreachSignalsPlot(Bpid,'H');
G.AddSignals('ref',1);
STL_ReadFile req_watertank.stl
R = BreachRequirement('phi')
R.Eval(Bpid);
BreachSamplesPlot(R)

%% 
pb_watertank.algo2();