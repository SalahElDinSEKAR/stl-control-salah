clear all
p= {'p1', 'p2'};
B1= BreachSet(p);
B1.SetParamRanges(p, [0 1]);
B1.SampleDomain(100);

B2= BreachSet(p);
B2.SetParamRanges(p, [0 2]);
B2.SampleDomain(100);

%%
close all
figure;
B1.PlotParams(p, [], 'sb');
B2.PlotParams(p, [], 'or');

%% Simple grid (5x5)
[Bg1, Bsub1] = B1.GridFilter(p, [5 5]);

%%
% Grid object 
%
%  G.is_covered_pt( )

pts2 = B2.GetParam(p);

covered = Bg1.check_covered_pts(p, pts2)

B2s = B2.ExtractSubset(find(covered));
B2s.PlotParams(p, [], 'xk');


