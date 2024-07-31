clear all
% One parameter
p= {'p1', 'p2'};
B1= BreachSet(p);
B1.SetParamRanges(p, [0 10]);
B1.SampleDomain(100);

%% Simple grid (5x5)

[Bg1, Bsub1] = B1.GridFilter(5);

%% 
close all
B1.PlotParams(p, [], 'b.');
Bg1.PlotParams(p, [], 'k.'); 
Bsub1.PlotParams(p, [], 'sr')

%% Custom grid

opts.params.p1.grid = [0 .5 5 9.5 10]; % specifies grid boundaries 
opts.params.p2.grid = [0 1 7 8 9 10];

B2= BreachSet(p);
B2.SetParamRanges(p, [0 10]);
B2.SampleDomain(100);

opts = B2.SetCoverageOptions(opts);
[Bg2, Bsub2] = B2.GridFilter(opts);

B2.GetCoverage(opts);
figure
B2.plot_cover_proj('p1__x__p2');

%% Signals

sg = random_signal_gen({'x1','x2'}, 'spline');
Bs = BreachSignalGen({sg});
Bs.SetParam('x2_seed',2);

Bs.Sim(0:.01:10)

