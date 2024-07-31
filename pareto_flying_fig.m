init_flying;
%load data/Res_Flying_Success15-03.mat

%%
init_sampling1000.num_corners= 256;
init_sampling1000.num_quasi_random = 744;
init_sampling1000.scaling = .8;
init_sampling1000.seed=5000;

%% MPC evaluation
[Bmpc, R] = pb_flying.get_mpc_traces(init_sampling1000);
Ry = BreachRequirement({'y[t]>0'}); 
og = expr_output_gen('y', 'sqrt(x1[t].^2 + x2[t].^2 + theta[t].^2 + v1[t]^2 + v2[t].^2 + omega[t].^2)');
Ry.AddPostProcess(og);
Ry.Eval(Bmpc);

%%
% tau_stable = [ 7:.5:14 ];
% p_stable =[];
% for tau = tau_stable
% tau
% B0 = Bmpc.copy();
% B0.SetParam('start_stable', tau, 1)
% R = BreachRequirement('phi_stable');    
% synth_pb = ParamSynthProblem(B0, R, {'p_stable'},  [0 5]);
% p_stable(end+1) = synth_pb.solve();
% end

%% Much faster 
load R1000.mat
figure;
list_nn = [  3 5 6 7 8  36];
hold on;

%% Figures 
close all;
h_max = figure; 
hold on;

% Max
[t1, ymean,t2, ymax ] = get_y_max(Ry);
%h_mean = figure;
subplot(1,2,1)    
plot(t1, ymean,'g', 'DisplayName','Nominal average');
hold on
legend
grid on
set(gca, 'XLim',[5 14]);


%h_max = figure;
subplot(1,2,2)    
plot(t2, ymax, 'g', 'DisplayName','Nominal max');
hold on
legend
grid on 
set(gca, 'XLim',[5 14]);

for inn = 1:numel(Rall)
    [t1, ymean, t2, ymax ] = get_y_max(Rall{inn});    
    subplot(1,2,1)    
    l_mean(inn) = plot(t1, ymean, 'DisplayName',['Average performance NN ' num2str(inn)], 'Visible', 'off');
    
    subplot(1,2,2)
    l_max(inn) = plot(t2, ymax, 'DisplayName',['Worst Performance NN ' num2str(inn)], 'Visible', 'off');    
end
legend()

set_visible(l_mean, list_nn)
set_visible(l_max, list_nn)




