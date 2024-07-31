%init_flying;
%load('C:\Users\alex\workspace\decyphir\AI\stl-control-imitation\data\Res_Flying_Paper1.mat');


%%  Performance for nominal
% Ry = pb_flying.getRy(all_results(1:14));
get_Ry_mpc = 1;

if (get_Ry_mpc)
    init_sampling100.num_corners= 16;
    init_sampling100.num_quasi_random = 84;
    init_sampling100.scaling = .8;
    init_sampling100.recompute=1; 
    init_sampling100.seed=10000;
    Bmpc = pb_flying.get_mpc_traces(init_sampling100);
    
    Ry_mpc = BreachRequirement({'y[t]>0'});
    og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
    Ry_mpc.AddPostProcess(og);
    Ry_mpc.Eval(Bmpc);
end

[t1_mpc, ymean_mpc,t2_mpc, yworst_mpc] = get_y_max(Ry_mpc);

%% Performance for NN 
get_Ry = 0; 
if get_Ry
    for ir = 1:numel(Ry)
        [t1{ir}, ymean{ir},t2{ir}, yworst{ir}] = get_y_max(Ry(ir));
    end
end
%[out,  l_worst, h_worst, l_mean, h_mean, Ry] = pb_flying.plot_pareto(all_results(1:14), Ry);

%% Plot p_over
p_over_mpc = yworst_mpc(1);
p_stab_mpc = yworst_mpc(end);
p_over  = [];
p_stab  = [];


t_pareto = 5:14;

y_mpc_pareto =  interp1(t2_mpc, yworst_mpc, t_pareto);
for ir = 1:numel(Ry)
    p_over(ir) = yworst{ir}(1);
    p_stab(ir) = yworst{ir}(end);    
    
    y_pareto =  interp1(t2{ir}, yworst{ir}, t_pareto);
    dist_pareto(ir) = norm(y_pareto- y_mpc_pareto);
end



%%
close all; 
figure;

plot(dist_pareto)


figure
lw = 1.5;
subplot(2,1,1);
%title('Overshoot Performance')
hold on
grid on;
iter = 1:numel(p_stab);
plot(iter, iter*0+p_over_mpc, '--r' ,'DisplayName','Nominal Controller', 'LineWidth', lw)
plot(iter, p_over,'b', 'DisplayName','Neural Net Controller', 'LineWidth', lw)
title('Overshoot vs number of training iterations')
 xlabel('Training iteration');
ylabel('s_{ov} (Overshoot)');
set(gca, 'XLim',[.9 14], 'XTick', 1:14);
legend
%
subplot(2,1,2);

title('Stabilization Performance')
%l_yworst_mpc = plot(t2_mpc, yworst_mpc);
pp = fill([t2_mpc fliplr(t2_mpc)], [yworst_mpc zeros(size(yworst_mpc))], 'r' , 'DisplayName','Nominal Non Sat.');
grid on
hold on;
%
for ir = 1:numel(Ry)
    l_yworst(ir)= plot(t2{ir}, yworst{ir}, 'DisplayName',['NN ' num2str(ir) ': \sigma=' num2str(dist_pareto(ir),3) ]);     
end
legend()
set(gca, 'XLim',[2 14]);
xlabel('\tau_{tr} (s) (Transient Time)');
ylabel('s_{st} (Stabilization Region) ' );
title('Stabilization Performance')
set_visible(l_yworst, [4 9 13 14])

set(l_yworst(4), 'LineStyle', '-.', 'LineWidth', lw)
set(l_yworst(9), 'LineStyle', '--', 'LineWidth', lw)
set(l_yworst(14), 'LineWidth', lw)

plot(14, 1.5, '.', 'MarkerSize',14, 'HandleVisibility','off')
text(14.2, 1.5,'$\overline{p}$', 'Interpreter','latex', 'FontSize',14)

%% Pareto front distance

fig_resize(gcf, 1,1.5)
save2pdf('fig3_pareto.pdf', gcf)

