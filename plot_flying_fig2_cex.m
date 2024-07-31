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
X0_style = {'FaceAlpha',0.1, 'FaceColor', 'k', 'DisplayName', 'Initial Set'};
set(pX0, X0_style{:})

%% Training grid 
Bt1 = r1.Btrain_grid;
Bp1 = Bt1.SimpleProject(params);
r_list = Bp1.plot_cover_grid(params, 'EdgeColor','w', 'HandleVisibility','off')
set(r_list(end), 'HandleVisibility','on', 'DisplayName', 'Cells with current training data');


%% Counter example traces 
cex_list = [2 4];

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

cex_style = {'LineStyle','--', 'Color', [0.5 0 0], 'Marker', '.'};
for ii = 1:numel(cex_list) 
    cex = sig_vals{ii};
    if ii == 1 
        l =plot(cex(1,:), cex(2,:),'DisplayName','NN control counter example traces');
    else
        l =plot(cex(1,:), cex(2,:), 'HandleVisibility','off');    
    end
        set(l, cex_style{:});    
end



%% plot fixed traces
fix_list =  1:50;
max_fix = 10;

Bfix= r1.Bcex_fix_traces;

fix_style = {'LineStyle','--', 'Color', [0 0.5 0], 'Marker', '.'};
sig_vals = Bfix.GetSignalValues(params);
if ~iscell(sig_vals)
    sig_vals ={sig_vals}
end

cex_keep_for_plot = [];
for ii = fix_list  
    fix = sig_vals{ii};
    x0 = [fix(1,1) ;fix(2,1)];
    if Bcex_select_grid.check_covered_pts(params, x0)&&~Bt1.check_covered_pts(params,x0)
        if isempty(cex_keep_for_plot)
           % plot(fix(1,1), fix(2,1), 'sg', 'HandleVisibility','off');
            l = plot(fix(1,:), fix(2,:), 'DisplayName','Nominal control new traces');
            cex_keep_for_plot(end+1) = ii;            
        else
           % plot(fix(1,1), fix(2,1), 'sk', 'HandleVisibility','off')
            l = plot(fix(1,:), fix(2,:), 'HandleVisibility','off');
            cex_keep_for_plot(end+1) = ii;            
        end

        if numel(cex_keep_for_plot)>max_fix
            break;
        end
        
        set(l, fix_style{:}); 
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
r_list = Bp_new.plot_cover_grid(params, 'FaceColor', 'g', 'EdgeColor', 'k', 'HandleVisibility','off');
set(r_list(end), 'HandleVisibility','on', 'DisplayName', 'Cells with new training data');

%% Legend
lg = legend();
%set(lg, 'Location','SouthEast')

%% Export
set(gca, 'XLim',[-8 8], 'YLim',[-8 8]);
%fig_resize(gcf, 1.5)
save2pdf('paper/One_iteration.pdf');








