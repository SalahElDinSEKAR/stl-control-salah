%% Illustrate one iteration, create figures

init_flying
load('C:\Users\alex\workspace\decyphir\AI\stl-control-imitation\data\Res_Flying_For_Fig.mat')

%params = {'v1','v2'};
params = {'x1','x2'};
%params = {'x1','x2','theta'};
init_list = [1:4 7:4:100];


%% plot Initial set + training
r1 = all_results{1};
Btraces1 = r1.Btrain_traces.ExtractSubset(init_list);
[Binit_samples, Binit_grid] = pb_flying.get_grid_data_from_traces(Btraces1);
sigvals = Btraces1.GetSignalValues(params);


%% Initial set
close all
figure; hold on;
Btraces1.PlotDomain({'x1_0', 'x2_0'});
set(gca, 'XLimMode','auto', 'YLimMode','auto');

pX0 = get(gca, 'Children');  % rectangle for initial set
X0_style = {'FaceAlpha',0.1, 'FaceColor', 'k'};
set(pX0, X0_style{:})

%text(-3.7,-3.5,0 , '$\cal{X}$', 'Interpreter','latex', 'FontSize',14)
%text(-3.2,-3.7,0 , '0', 'Interpreter','latex', 'FontSize',10)

%% Initial traces
nom_trace_style = {'LineStyle','--', 'Color', [0 0.5 0], 'Marker', '.'};
%Btraces1.PlotParams(params)

for ii =1:numel(sigvals)
    nom_trace = sigvals{ii};    
    if ii==1
        plot(nom_trace(1,1), nom_trace(2,1), 'sk');
        l = plot(nom_trace(1,:), nom_trace(2,:));
        lg = legend('Initial Set', 'Initial Samples', 'Nominal control traces');
        set(lg, 'Location','SouthEast')
    else
        plot(nom_trace(1,1), nom_trace(2,1), 'sk', 'HandleVisibility','off');    
        l = plot(nom_trace(1,:), nom_trace(2,:), 'HandleVisibility','off');    
    end
    set(l, nom_trace_style{:});    

end

Bp1 = Binit_grid.SimpleProject(params);
%Binit_samples.PlotParams(params)
r_list =Bp1.plot_cover_grid(params, 'EdgeColor','w',  'HandleVisibility','off');

set(r_list(end), 'HandleVisibility','on', 'DisplayName', 'Cells covered by traces');

%% Export 
set(gca, 'XLim',[-8 8], 'YLim',[-8 8]);
%fig_resize(gcf, 1.5)
save2pdf('paper/Initial_sampling.pdf');
