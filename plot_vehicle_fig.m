function plot_vehicle_fig(res)


init_vehicle;
B0 = pb_vehicle.create_nominal();

p = {'x1_0','x2_0'};
s =  {'x1','x2'};

%B0.PlotDomain(p)
%res.Btrain.PlotParams(s, [], {'sk'});

pb_vehicle.plot_traces(res.Bcex_nn_traces)
res.Bcex_fix_traces.PlotSigPortrait(s,[], {'g','LineWidth',1} );




end
