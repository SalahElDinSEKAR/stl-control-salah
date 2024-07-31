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

%save2pdf('Initial_sampling.pdf');
return

%% Plot one iteration
cex_list = [2 4];
fix_list =  1:50;
max_fix = 10;


%% Plot current training grid 
Bt1 = r1.Btrain_grid;
Bp1 = Bt1.SimpleProject(params);

%%
close all;
figure;
Btraces1.PlotDomain({'x1_0', 'x2_0'});
set(gca, 'XLimMode','auto', 'YLimMode','auto');
pX0 = get(gca, 'Children');  % rectangle for initial set
set(pX0, X0_style{:})
%%

Bp1.plot_cover_grid(params, 'EdgeColor','w')


%% plot worst counter example traces 
Bcex= r1.Bcex_nn_traces;

Bcex_select = Bcex.ExtractSubset(cex_list);
[~, Bcex_select_grid] = pb_flying.get_grid_data_from_traces(Bcex_select);

%Bcex_select_proj = Bcex_select_grid.SimpleProject(params);
%Bcex_select_proj.plot_cover_grid(params, 'EdgeColor','r')
%pause;

sig_vals = Bcex_select.GetSignalValues(params);
if ~iscell(sig_vals)
    sig_vals ={sig_vals}
end
hold on;

for ii = 1:numel(cex_list) 
    cex = sig_vals{ii};
    plot(cex(1,:), cex(2,:), 'r');    
end


%% plot fixed traces
Bfix= r1.Bcex_fix_traces;

sig_vals = Bfix.GetSignalValues(params);
if ~iscell(sig_vals)
    sig_vals ={sig_vals}
end

cex_keep_for_plot = [];
for ii = fix_list  
    fix = sig_vals{ii};
    x0 = [fix(1,1) ;fix(2,1)];
    if Bcex_select_grid.check_covered_pts(params, x0)&&~Bt1.check_covered_pts(params,x0)
        plot(fix(1,:), fix(2,:), '+-g');
        drawnow
        cex_keep_for_plot(end+1) = ii;

        pause
        if numel(cex_keep_for_plot)>max_fix
            break;
        end
    end
end

%% plot new training set from select traces
%r2 = all_results{2};
%Bt2 = r2.Btrain_grid;

Bfix_select= Bfix.ExtractSubset(cex_keep_for_plot);
[~, Bt2 ] = pb_flying.get_grid_data_from_traces(Bfix_select)
Bp2 = Bt2.SimpleProject(params)
val1 = Bp1.GetParam(params);
Bp_new = Bp2.copy();
Bp_new.RemoveParams(params, val1)


%%
hold on;
Bp_new.plot_cover_grid(params, 'FaceColor', 'r', 'EdgeColor', 'w');

%%










