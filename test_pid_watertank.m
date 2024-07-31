clear all;
init_watertank;

B0 = pb_watertank.create_nominal();

B0.Sim();

%%
BreachSignalsPlot(B0, {'H', 'ref'});
%%

STL_ReadFile req_watertank.stl
R = BreachRequirement('phi')
R.Eval(B0);
BreachSamplesPlot(R)