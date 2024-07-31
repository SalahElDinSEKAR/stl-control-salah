clear all;
close all;
%% Signals
sg = random_signal_gen({'x1','x2'}, 'spline');
Bs = BreachSignalGen({sg});
Bs.SetParam('x2_seed',2); % makes x2 different from x1 (?)
Bs.Sim(0:.01:10)
Bs.PlotSignals;
if 0
%%
[Bg, Bf, Ba ] = Bs.GridFilterSignals({'time','x1'},[3 .1]);

%%
close all;
figure
Bs.PlotSignals('x1')
Bg.PlotBoxPts({'time', 'x1'});  % plots the grid cells covered by Bs in time and x1
Bf.PlotParams({'time', 'x1'},[], 'rs')  % plots the samples filtered, one per grid cell
Ba.PlotParams({'time', 'x1'},[], 'bx')  % plots the samples filtered, one per grid cell
%view([45 25]) % Bsub1 retains y values

%% Compute Bf_sorted

pts = Bg.DeltaGridMapObj.values;
for i = 1:numel(pts)
    pt = pts{i};
    indices = pt.idx;
    val_for_sorting = Ba.GetParam('x1', indices);
    [~, this_indices_sorted] = sort(val_for_sorting);
    indices_max(i) = indices(this_indices_sorted(end));
end

Bf_sorted = Ba.ExtractSubset(indices_max);
Bf_sorted.PlotParams({'time', 'x1'},[], 'ks')  % plots the samples filtered, one per grid cell
end
%%
opts.signals.x1.range = [-1 1];
opts.signals.x1.grid = .3;
opts.signals.x2.range = [-2 2];
opts.signals.x2.grid = .5;


[Bf, Bg, Ba ] = Bs.GridFilterSignals({'x1','x2'},opts );
figure;
Bf.PrintAll;
Ba.PlotParams({'x1','x2'},[],'.b')
Bf.PlotParams({'x1','x2'},[], 'r+')

