clear all
p= {'x1', 'x2', 'y'};
B1= BreachSet(p);
B1.SetParamRanges(p, [0 1]);
B1.SampleDomain(100);

%% Simple grid (5x5)
[Bg1, Bsub1] = B1.GridFilter({'x1', 'x2'}, [5 5]);

%%
Bg1.PlotBoxPts();  % plots the grid cells covered by B1
Bsub1.PlotParams(p,[], 'bx')  % plots the samples filtered, one per grid cell
 view([45 25]) % Bsub1 retains y values

%% 
close all
B1.PlotParams(p, [], 'b.');
Bg1.PlotParams(p, [], 'ks'); 
Bsub1.PlotParams(p, [], 'sr')

%% Custom grid

opts.params.x1.grid = [0 1e-3 1e-2 1e-1 1 10]; % specifies grid 
opts.params.x2.grid = [0 1e-3 1e-2 1e-1 1 10]; % specifies grid 
B2= BreachSet(p);
B2.SetParamRanges(p, [0 10]);
B2.SampleDomain(100);

opts = B2.SetCoverageOptions(opts, 'ExcludeParams',{'y'} );
[Bg2, Bsub2] = B2.GridFilter(opts);

%%
B2.GetCoverage(opts);
figure
B2.plot_cover_proj('x1__x__x2');
