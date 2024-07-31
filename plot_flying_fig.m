function plot_flying_fig(res)

init_flying_problem;
B0 = pb_flying.create_nominal();

p = {'x1_0','x2_0','theta_0'};
s =  {'x1','x2','theta'};

%B0.PlotDomain(p)
%res.Btrain.PlotParams(s, [], {'sk'});

res.Bcex_nn_traces.PlotSigPortrait(s,1, {'r','LineWidth',2} );
res.Bcex_fix_traces.PlotSigPortrait(s,[], {'g','LineWidth',2} );

end